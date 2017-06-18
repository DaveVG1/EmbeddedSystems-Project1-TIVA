//*****************************************************************************
//
// Practica 1.
// Autor: David Vidriales Guerrero
//
//*****************************************************************************

#include<stdbool.h>
#include<stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "drivers/buttons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "inc/hw_adc.h"
#include "utils/cpu_usage.h"

#include "drivers/rgb.h"
#include "usb_dev_serial.h"
#include "protocol.h"
#include "remote.h"

#include "event_groups.h"

#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128

#define PB0_FLAG 0x00000001
#define PB1_FLAG 0x00000002
#define PB2_FLAG 0x00000004
#define PB3_FLAG 0x00000008
#define AIN0_FLAG 0x00000010
#define AIN1_FLAG 0x00000020
#define AIN2_FLAG 0x00000040
#define AIN3_FLAG 0x00000080

//Globales

uint32_t g_ui32CPUUsage;
uint32_t g_ulSystemClock;

// Colas
xQueueHandle cola_buttons;
xQueueHandle cola_freertos;

// EventGroups
EventGroupHandle_t EventADC;

PARAM_COMANDO_ADC parametro; // Variable global -> intentar eliminar

extern void vUARTTask( void *pvParameters );


//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	//
	// This function can not return, so loop forever.  Interrupts are disabled
	// on entry to this function, so no processor interrupts will interrupt
	// this loop.
	//
	while(1)
	{
	}
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
	static unsigned char count = 0;

	if (++count == 10)
	{
		g_ui32CPUUsage = CPUUsageTick();
		count = 0;
	}
	//return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
	SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
	while(1);
}



//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************
//*****************************************************************************
//
// Codigo para procesar los datos para poder implementar el Osciloscopio, se
// reciben los datos desde una interrupción causada por el Timer2A, interrupción
// tipo HW.
// Se reciben los datos desde la interrupción a traves de las colas, se espera
// hasta tener al menos 40 muestras(10 por canal) y posteriormente se mandan por
// USB al PC.
// Falta DMA
//
//*****************************************************************************
static portTASK_FUNCTION(OsciloscopioTask, pvParameters ){


	unsigned char frame[MAX_FRAME_SIZE];	//Ojo, esto hace que esta tarea necesite bastante pila
	int numdatos, command=COMANDO_RECIBIR_OSCILOSCOPIO;
	PARAM_COMANDO_RECIBIR_OSCILOSCOPIO parametro;
	int i;
	while(1)
	{
		for (i=0;i<10;i++){
			xQueueReceive(cola_freertos,&parametro.valores.valor0[i],portMAX_DELAY);
			xQueueReceive(cola_freertos,&parametro.valores.valor1[i],portMAX_DELAY);
			xQueueReceive(cola_freertos,&parametro.valores.valor2[i],portMAX_DELAY);
			xQueueReceive(cola_freertos,&parametro.valores.valor3[i],portMAX_DELAY);/**/
		}

		numdatos=create_frame(frame,command,&parametro, sizeof(parametro),MAX_FRAME_SIZE*2);
		if (numdatos>=0){
			send_frame(frame,numdatos);
		} else {
			// Error
		}
	}
}

// Tarea para la gestión del envío asíncrono de las alarmas provocadas por las entradas analógicas y digitales
// Mientras que no haya ninguna alarma activada se queda esperando
// Cuando se produce la primera alarma, entramos en un bucle que actualiza las alarmas hasta que no estén todas desactivadas
// Tiene varios fallos:
// 1. Cuando se desactiva la última alarma, no se desactiva en QT exactamente a los 10, sino que se desactiva entre 5 y 10 s
// Esto es debido a el último envío de alarma que se produce cada 5 segundos, puede haberse mandado entre los 0 y 5 primeros
// segundos que espera QT
// 2. Las entradas analógicas no funcionan correctamente, las digitales por si solas si
//*****************************************************************************

static portTASK_FUNCTION(ADCTask,pvParameters)
{
	uint32_t flags, flags_aft = 0;
	unsigned char frame[MAX_FRAME_SIZE];
	int numdatos;
	uint32_t ui32Period;
	uint16_t cuenta = 0; // Variable para ver si sólo hay una única entrada activa
	uint16_t ok = 0; // Variable que sólo se activa una vez para activar el TIMER4
	ui32Period = SysCtlClockGet() * 5; // Cargamos 5 s en el TIMER4
	while(1)
	{
		//Espera a que se active algún flag
		// Al activarse, no lo borra y continua
		flags = xEventGroupWaitBits(EventADC, PB0_FLAG|PB1_FLAG|PB2_FLAG|PB3_FLAG|AIN0_FLAG|AIN1_FLAG|AIN2_FLAG|AIN3_FLAG , pdFALSE, pdFALSE, portMAX_DELAY);
		ok = 1;
		while (flags > 0) { // Mientras que sigamos activos enviamos alarma
			flags = xEventGroupGetBits(EventADC);
			if(flags_aft != flags) { // Si algún flag ha cambiado debo enviar mensaje inmediatamente y reiniciar el timer
				flags_aft = flags;
				if (flags & PB0_FLAG) {
					if(parametro.din.PB0 == 0) { // Si hay un cambio, cambio la variable y actualizo la cuenta si no, no
						parametro.din.PB0 = 1;
						++cuenta;
					}
				} else if (!(flags & PB0_FLAG)) {
					if(parametro.din.PB0 == 1) {
						parametro.din.PB0 = 0;
						--cuenta;
					}
				}

				if (flags & PB1_FLAG) {
					if(parametro.din.PB1 == 0) { // Si hay un cambio, cambio la variable y actualizo la cuenta si no, no
						parametro.din.PB1 = 1;
						++cuenta;
					}
				} else if (!(flags & PB1_FLAG)) {
					if(parametro.din.PB1 == 1) {
						parametro.din.PB1 = 0;
						--cuenta;
					}
				}

				if (flags & PB2_FLAG) {
					if(parametro.din.PB2 == 0) { // Si hay un cambio, cambio la variable y actualizo la cuenta si no, no
						parametro.din.PB2 = 1;
						++cuenta;
					}
				} else if (!(flags & PB2_FLAG)) {
					if(parametro.din.PB2 == 1) {
						parametro.din.PB2 = 0;
						--cuenta;
					}
				}

				if (flags & PB3_FLAG) {
					if(parametro.din.PB3 == 0) { // Si hay un cambio, cambio la variable y actualizo la cuenta si no, no
						parametro.din.PB3 = 1;
						++cuenta;
					}
				} else if (!(flags & PB3_FLAG)) {
					if(parametro.din.PB3 == 1) {
						parametro.din.PB3 = 0;
						--cuenta;
					}
				}

				if (flags & AIN0_FLAG) {
					if(parametro.anin.AIN0 == 0) {
						parametro.anin.AIN0 = 1;
						++cuenta;
					}
				} else if (!(flags & AIN0_FLAG)) {
					if(parametro.anin.AIN0 == 1) {
						parametro.anin.AIN0 = 0;
						--cuenta;
					}
				}

				if (flags & AIN1_FLAG) {
					if(parametro.anin.AIN1 == 0) {
						parametro.anin.AIN1 = 1;
						++cuenta;
					}
				} else if (!(flags & AIN1_FLAG)) {
					if(parametro.anin.AIN1 == 1) {
						parametro.anin.AIN1 = 0;
						--cuenta;
					}
				}

				if (flags & AIN2_FLAG) {
					if(parametro.anin.AIN2 == 0) {
						parametro.anin.AIN2 = 1;
						++cuenta;
					}
				} else if (!(flags & AIN2_FLAG)) {
					if(parametro.anin.AIN2 == 1) {
						parametro.anin.AIN2 = 0;
						--cuenta;
					}
				}

				if (flags & AIN3_FLAG) {
					if(parametro.anin.AIN3 == 0) {
						parametro.anin.AIN3 = 1;
						++cuenta;
					}
				} else if (!(flags & AIN3_FLAG)) {
					if(parametro.anin.AIN3 == 1) {
						parametro.anin.AIN3 = 0;
						--cuenta;
					}
				}
				if (cuenta != 0) {
					// Envío de la información del estado de las entradas analógicas
					numdatos = create_frame(frame, COMANDO_ADC, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
					if (numdatos >= 0){
						send_frame(frame, numdatos);
					}

					if(ok) {
						ok = 0;
						// Carga la cuenta en el Timer4A
						TimerLoadSet(TIMER4_BASE, TIMER_A, ui32Period -1);
						// Empieza a funcionar
						TimerEnable(TIMER4_BASE, TIMER_A);
					}
				} else {
					// No hago nada, en la próxima iteración saldremos del while y desactivaremos el TIMER4
				}
			} else {
				// No hago nada, el timer enviará el mensaje
			}
		}
		TimerDisable(TIMER4_BASE, TIMER_A); // Cuando no haya flags activos desactivo la interrupción del timer
	}
}
// Tarea para la gestión del envío asíncrono de la pulsación de botones
//*****************************************************************************

static portTASK_FUNCTION(ButtonsTask, pvParameters){

	//Datos necesarios para el envío necesario de los datos por USB
	unsigned char frame[MAX_FRAME_SIZE];
	int numdatos, command = COMANDO_SWITCH;
	PARAM_COMANDO_SWITCHES parametro;

	uint8_t i32PinStatus; // Variable donde se almacenará el estado de los botones

		while(1){
			xQueueReceive(cola_buttons, &i32PinStatus, portMAX_DELAY); // Recibe el estado de los botones

			// Procesamiento de la información de los botones
			if ((i32PinStatus & LEFT_BUTTON)){
				parametro.switches.SW1 = true;
			} else {
				parametro.switches.SW1 = false;
			}
			if ((i32PinStatus & RIGHT_BUTTON)){
				parametro.switches.SW2 = true;
			} else {
				parametro.switches.SW2 = false;
			}

			// Envío de la información del estado de los botones
			numdatos = create_frame(frame, command, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
			if (numdatos >= 0){
				send_frame(frame, numdatos);
			}
			GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS); // Limpia el flag de interrupción causado por la interrupción
		}
}

// El codigo de esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos del interprete a traves
// del terminal serie (puTTY)
//Aqui solo la declaramos para poderla referenciar en la funcion main
extern void vUARTTask( void *pvParameters );

//*****************************************************************************
//
// Funcion main(), Inicializa los perifericos, crea las tareas, etc... y arranca el bucle del sistema
//
//*****************************************************************************
int main(void)
{
	memset(&parametro, 0, sizeof(parametro));
	//
	// Set the clocking to run at 40 MHz from the PLL.
	//
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
			SYSCTL_OSC_MAIN);	//Ponermos el reloj principal a 40 MHz (200 Mhz del Pll dividido por 5)


	// Get the system clock speed.
	g_ulSystemClock = SysCtlClockGet();


	//Habilita el clock gating de los perifericos durante el bajo consumo --> perifericos que se desee activos en modo Sleep
	//                                                                        deben habilitarse con SysCtlPeripheralSleepEnable
	ROM_SysCtlPeripheralClockGating(true);

	// Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
	// Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
	// (y por tanto este no se deberia utilizar para otra cosa).
	CPUUsageInit(g_ulSystemClock, configTICK_RATE_HZ/10, 3);

	//
	// Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
	//se usa para mandar y recibir mensajes y comandos por el puerto serie
	// Mediante un programa terminal como gtkterm, putty, cutecom, etc...
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0,115200,SysCtlClockGet());

	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);	//La UART tiene que seguir funcionando aunque el micro este dormido
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);	//La UART tiene que seguir funcionando aunque el micro este dormido


	//Inicializa los LEDs usando libreria RGB --> usa Timers 0 y 1 (eliminar si no se usa finalmente)
	RGBInit(1);
	SysCtlPeripheralSleepEnable(GREEN_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(BLUE_TIMER_PERIPH);
	SysCtlPeripheralSleepEnable(RED_TIMER_PERIPH);	//Redundante porque BLUE_TIMER_PERIPH y GREEN_TIMER_PERIPH son el mismo

	//Inicializa los botones
	ButtonsInit();
	ROM_GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_BOTH_EDGES);
	GPIOIntEnable(GPIO_PORTF_BASE, ALL_BUTTONS);

	// Parte 2

	// ADC

	// Habilitamos el periférico del ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

	//HABILITAMOS EL GPIOE
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
	// Enable pin PE3 for ADC AIN0|AIN1|AIN2|AIN3
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);

	//CONFIGURAR SECUENCIADOR 0
	ADCSequenceDisable(ADC0_BASE, 0);

	//Configuramos la velocidad de conversion al maximo (1MS/s)
	ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

	// Configuramos el disparo por comparador
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_ALWAYS, 0);	//Disparo software (processor trigger)
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CMP0 | ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CMP1 | ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CMP2 | ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CMP3 | ADC_CTL_CH3);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CMP4 | ADC_CTL_END );	// La última muestra provoca la interrupcion
	ADCSequenceEnable(ADC0_BASE, 0); // ACTIVO LA SECUENCIA

	// Habilitamos el GPIOB
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
	ROM_GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
	ROM_GPIOIntTypeSet(GPIO_PORTB_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0 ,GPIO_BOTH_EDGES);
	GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_1|GPIO_PIN_0);
	//ROM_IntEnable(INT_GPIOB); -> Lo hacemos desde el comando recibido

	// Configuracion TIMER4 -> Se encarga de mandar el mensaje de alarma cada 5 s
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER4);
	// Cuenta periódica
	TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
	// Habilita interrupcion del modulo TIMER
	IntEnable(INT_TIMER4A);
	// Y habilita, dentro del modulo TIMER4, la interrupcion de particular de "fin de cuenta"
	TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

	// Osciloscopio

	// Configurar ADC

	ADCSequenceDisable(ADC0_BASE, 1); // Deshabilita el secuenciador 1 del ADC0 para su configuracion

	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_TIMER, 0);//1
	ADCHardwareOversampleConfigure(ADC0_BASE,10);//Sobremuestreo para realizar el promediado de 16 muestras
	// Configuramos los 4 conversores del secuenciador 1
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0 );
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1 );
	// El conversor 4 es el último, y  se genera un aviso de interrupcion
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH3  | ADC_CTL_IE | ADC_CTL_END);//3
	// Este tiene que estar aquí.
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH2 );

	// Tras configurar el secuenciador, se vuelve a habilitar
	ADCSequenceEnable(ADC0_BASE, 1);
	// bucle infinito del programa principal

	//IntEnable(INT_ADC0SS1);// Habilitación a nivel global del sitema
	ADCIntEnable(ADC0_BASE,1);//Habilitación del secuenciador dentro del periférico
	ADCIntClear(ADC0_BASE,1);//Borramos posibles interrupciones pendientes

	// Configura TIMER2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	TimerControlTrigger(TIMER2_BASE,TIMER_A,1);
	IntPrioritySet(INT_TIMER2A,6);
	IntEnable(INT_TIMER2A);
	TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT);

	// Reloj

	// Configuracion TIMER5 -> Se encarga de actualizar el reloj cada segundo
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER5);
	ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER5);
	// Cuenta periódica
	TimerConfigure(TIMER5_BASE, TIMER_CFG_PERIODIC);
	// Habilita interrupcion del modulo TIMER
	IntEnable(INT_TIMER5A);
	// Y habilita, dentro del modulo TIMER4, la interrupcion de particular de "fin de cuenta"
	TimerIntEnable(TIMER5_BASE, TIMER_TIMA_TIMEOUT);

	// Grupos
	EventADC = xEventGroupCreate();
	if(EventADC == NULL)
	{
		while(1);
	}
	// Colas
	cola_buttons = xQueueCreate(1, sizeof(uint32_t));
	if (NULL == cola_buttons)
		while(1); // Si hay problemas para crear la cola, se queda aquí.
	cola_freertos=xQueueCreate(4,sizeof(uint8_t)); //Aquí debemos poner 16 o 64??
	if (NULL==cola_freertos)
		while(1); // Si hay problemas para crear la cola, se queda aquí.

	//
	// Mensaje de bienvenida inicial.
	//
	UARTprintf("\n\nBienvenido a la aplicacion XXXXX (curso 2016/17)!\n");
	UARTprintf("\nAutor: David Vidriales Guerrero ");

	/**                                              Creacion de tareas 										**/

	//SEMANA 2: Lanza una tarea para leer y enviar los datos del ADC

	if((xTaskCreate(ADCTask, (portCHAR *)"ADC", 256,NULL,tskIDLE_PRIORITY + 2, NULL) != pdTRUE))
	{
		while(1);
	}

	if(xTaskCreate(OsciloscopioTask, (portCHAR *)"Osciloscopio",512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
	{
		while(1);
	}

	// Crea la tarea que gestiona los comandos UART (definida en el fichero commands.c)
	//
	if((xTaskCreate(vUARTTask, (portCHAR *)"Uart", 512,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE))
	{
		while(1);
	}

	UsbSerialInit(32,32);	//Inicializo el  sistema USB
	RemoteInit(); //Inicializo la aplicacion de comunicacion con el PC (Remote)


	if(xTaskCreate(ButtonsTask, (portCHAR *)"Buttons", 512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE)
	{
			while(1);
	}

	//
	// Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
	//
	vTaskStartScheduler();	//el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

	//De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
	while(1)
	{
		//Si llego aqui es que algo raro ha pasado
	}
}

// Rutinas de interrupcion
void GPIOFIntHandler(void){
	//Lee el estado del puerto (activos a nivel bajo)
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	int32_t i32PinStatus = ROM_GPIOPinRead(GPIO_PORTF_BASE, ALL_BUTTONS);
	xQueueSendFromISR(cola_buttons, &i32PinStatus, &xHigherPriorityTaskWoken);

	GPIOIntClear(GPIO_PORTF_BASE, ALL_BUTTONS);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void GPIOBIntHandler(void){
	//Lee el estado del puerto (activos a nivel bajo)
	BaseType_t higherPriorityTaskWoken = pdFALSE;

	int32_t i32PinStatus;
	i32PinStatus = ROM_GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	if(i32PinStatus & GPIO_PIN_0) {
		xEventGroupSetBitsFromISR(EventADC, PB0_FLAG , &higherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0);
	} else if (!(i32PinStatus & GPIO_PIN_0)){
		xEventGroupClearBitsFromISR(EventADC, PB0_FLAG);
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0);
	}
	if (i32PinStatus & GPIO_PIN_1) {
		xEventGroupSetBitsFromISR(EventADC, PB1_FLAG , &higherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_1);
	} else if (!(i32PinStatus & GPIO_PIN_1)){
		xEventGroupClearBitsFromISR(EventADC, PB1_FLAG);
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_1);
	}
	if (i32PinStatus & GPIO_PIN_2) {
		xEventGroupSetBitsFromISR(EventADC, PB2_FLAG , &higherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2);
	} else if (!(i32PinStatus & GPIO_PIN_2)){
		xEventGroupClearBitsFromISR(EventADC, PB2_FLAG);
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_2);
	}
    if (i32PinStatus & GPIO_PIN_3) {
		xEventGroupSetBitsFromISR(EventADC, PB3_FLAG , &higherPriorityTaskWoken );
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_3);
	} else if (!(i32PinStatus & GPIO_PIN_3)){
		xEventGroupClearBitsFromISR(EventADC, PB3_FLAG);
		GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_3);
	}
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void IntOsciloscopio(void){

	uint32_t ui32ADC0Value[4];
	uint8_t ui8Convertido[4];
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	ADCIntClear(ADC0_BASE, 1); // Limpia el flag de interrupcion del ADC

	// Tras haber finalizado la conversion, leemos los datos del secuenciador a un array
	//Lee valor de las muestras
	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
	ui8Convertido[0] = ui32ADC0Value[0]>>4;
	ui8Convertido[1] = ui32ADC0Value[1]>>4;
	ui8Convertido[2] = ui32ADC0Value[2]>>4;
	ui8Convertido[3] = ui32ADC0Value[3]>>4;

   xQueueSendFromISR(cola_freertos,&ui8Convertido[0],&xHigherPriorityTaskWoken);//Escribe en la cola freeRTOS
   xQueueSendFromISR(cola_freertos,&ui8Convertido[1],&xHigherPriorityTaskWoken);//Escribe en la cola freeRTOS
   xQueueSendFromISR(cola_freertos,&ui8Convertido[2],&xHigherPriorityTaskWoken);//Escribe en la cola freeRTOS
   xQueueSendFromISR(cola_freertos,&ui8Convertido[3],&xHigherPriorityTaskWoken);//Escribe en la cola freeRTOS

   TimerIntClear(TIMER2_BASE,TIMER_TIMA_TIMEOUT);
   portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	//Almacena las muestras en una cola FreeRTOS
	//Envío de las muestras por USB al PC
	//Apuntar esta interrupción en la tabla de vectores de interrupción en la entrada del ADC (0), secuenciador 1
}

void IntAlarma(void)
{
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	unsigned char frame[MAX_FRAME_SIZE];
	int numdatos;
	numdatos = create_frame(frame, COMANDO_ADC, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
	if (numdatos >= 0){
		send_frame(frame, numdatos);
	}
	// Borra la interrupcion de Timer
	TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void ADCIntHandler(void)
{
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;
	uint32_t i32Status;

	i32Status = ADCComparatorIntStatus(ADC0_BASE);

// Comprobamos qué entradas han producido la interrupción
	// Cambiamos el FLAG, dependiendo del estado anterior, ya que tenemos interrupción tanto para HIGH (activo FLAG)
	// como para LOW (desactivo FLAG)
	if(i32Status & 3) {
		if(parametro.anin.AIN0 == 0) {
			xEventGroupSetBitsFromISR(EventADC, AIN0_FLAG , &higherPriorityTaskWoken );
		} else if (parametro.anin.AIN0 == 1) {
			xEventGroupClearBitsFromISR(EventADC, AIN0_FLAG);
		}
	}

	if(i32Status & 4) {
		if(parametro.anin.AIN1 == 0) {
			xEventGroupSetBitsFromISR(EventADC, AIN1_FLAG , &higherPriorityTaskWoken );
		} else if (parametro.anin.AIN1 == 1) {
			xEventGroupClearBitsFromISR(EventADC, AIN1_FLAG);
		}
	}

	if(i32Status & 8) {
		if(parametro.anin.AIN2 == 0) {
			xEventGroupSetBitsFromISR(EventADC, AIN2_FLAG , &higherPriorityTaskWoken );
		} else if (parametro.anin.AIN2 == 1) {
			xEventGroupClearBitsFromISR(EventADC, AIN2_FLAG);
		}
	}

	if(i32Status & 16) {
		if(parametro.anin.AIN3 == 0) {
			xEventGroupSetBitsFromISR(EventADC, AIN3_FLAG , &higherPriorityTaskWoken );
		} else if (parametro.anin.AIN3 == 1) {
			xEventGroupClearBitsFromISR(EventADC, AIN3_FLAG);
		}
	}


	ADCComparatorIntClear(ADC0_BASE, i32Status);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

void IntReloj(void)
{
	portBASE_TYPE higherPriorityTaskWoken = pdFALSE;

	if(sec+1 == 60) {
		sec = 0;
		if(min+1 == 60) {
			min = 0;
			if(hora+1 == 24) {
				hora = 0;
			} else {
				hora++;
			}
		} else {
			min++;
		}
	} else {
		sec++;
	}

	TimerIntClear(TIMER5_BASE,TIMER_TIMA_TIMEOUT);
	portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}
