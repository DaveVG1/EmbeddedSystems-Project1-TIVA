/*
 * remote.c
 *
 *  Created on: 1/4/2016
 *      Author: jcgar
 */

#include"remote.h"

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
#include "drivers/rgb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

static uint8_t frame[MAX_FRAME_SIZE];	//Usar una global permite ahorrar pila en la tarea, pero hay que tener cuidado!!!!
static uint32_t gRemoteProtocolErrors=0;

//Defino a un tipo que es un puntero a funcion con el prototipo que tienen que tener las funciones que definamos
typedef int32_t (*remote_fun)(uint32_t param_size, void *param);


//Funcion que se ejecuta cuando llega un paquete indicando comando rechazado
int32_t ComandoRechazadoFun(uint32_t param_size, void *param)
{
	//He recibido "Comando rechazado" desde el PC
	//TODO, por hacer: Tratar dicho error??
	gRemoteProtocolErrors++;
	return 0;
}


//Funcion que se ejecuta cuando llega un PING
int32_t ComandoPingFun(uint32_t param_size, void *param)
{
	int32_t numdatos;

	numdatos=create_frame(frame,COMANDO_PING,0,0,MAX_FRAME_SIZE);
	if (numdatos>=0)
	{
		send_frame(frame,numdatos);
	}

	return numdatos;
}


//Funcion que se ejecuta cuando llega el comando que configura los LEDS
int32_t ComandoLedsFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_LEDS parametro;

	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
	{

		if(parametro.leds.red) {
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
		} else {
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
		}

		if(parametro.leds.green) {
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
		} else {
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0);
		}

		if(parametro.leds.blue) {
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
		} else {
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
		}

		return 0;	//Devuelve Ok (valor mayor no negativo)
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

int32_t ComandoRGBFun(uint32_t param_size, void *param)
{

	PARAM_COMANDO_LEDS parametro;
	uint32_t ulColors[3] = {0x0000, 0x0000, 0x0000};

	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
	{
		ulColors[0]= parametro.leds.red*257;
		ulColors[1]=parametro.leds.green*257;
		ulColors[2]= parametro.leds.blue*257;

		RGBColorSet(ulColors);

		return 0;	//Devuelve Ok (valor mayor no negativo)
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

//Funcion que se ejecuta cuando recibimos un comando que no tenemos aun implementado
int32_t ComandoNoImplementadoFun(uint32_t param_size, void *param)
{
	return PROT_ERROR_UNIMPLEMENTED_COMMAND; /* Devuelve un error para que lo procese la tarea que recibe los comandos */
}

//Funcion que se ejecuta cuando llega el comando que configura los LEDS
int32_t ComandoBrilloLedsFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_BRILLO parametro;


	if (check_and_extract_command_param(param, param_size, sizeof(parametro),&parametro)>0)
	{

		RGBIntensitySet(parametro.rIntensity);

		return 0;	//Devuelve Ok (valor mayor no negativo)
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

int32_t ComandoSwitchFun(uint32_t param_size, void *param)
{
	int32_t numdatos;

	PARAM_COMANDO_SWITCHES parametro;

	uint8_t ui8Changed, ui8Buttons;
	ButtonsPoll(&ui8Changed, &ui8Buttons);

	if (!(ui8Buttons & LEFT_BUTTON)){
		parametro.switches.SW1 = true;
	} else {
		parametro.switches.SW1 = false;
	}
	if (!(ui8Buttons & RIGHT_BUTTON)){
		parametro.switches.SW2 = true;
	} else {
		parametro.switches.SW2 = false;
	}

	numdatos=create_frame(frame, COMANDO_SWITCH, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
	if (numdatos >= 0) {
		send_frame(frame, numdatos);
	}

	return numdatos;
}

int32_t ComandoModeFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_MODE parametro;
	if(check_and_extract_command_param(param, param_size, sizeof(parametro), &parametro) > 0)
	{
		if (parametro.mode == 0)
		{
			RGBEnable();

			return 0; //Devuelve Ok (valor mayor no negativo)
		}
		else if (parametro.mode == 1)
		{
			RGBDisable();
			ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

			return 0; //Devuelve Ok (valor mayor no negativo)
		}
		else
		{
			return PROT_ERROR_UNIMPLEMENTED_COMMAND; // Comando no implementado
		}
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

int32_t ComandoButtonsIntFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_BUTTONSINT parametro;
	if (check_and_extract_command_param(param, param_size, sizeof(parametro), &parametro) > 0)
	{
		if(parametro.buttonsint.OK) {
			ROM_IntEnable(INT_GPIOF);
		} else {
			ROM_IntDisable(INT_GPIOF);
		}

		return 0; //Devuelve Ok (valor mayor no negativo)
	}
	else
	{
		return PROT_ERROR_INCORRECT_PARAM_SIZE; //Devuelve un error
	}
}

//SEMANA2: Funcion que procesa el comando ADC
int32_t ComandoADCFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_ADC parametro;

	if (check_and_extract_command_param(param, param_size, sizeof(parametro), &parametro) > 0) {
		if(parametro.ok){

			IntPrioritySet(INT_ADC0SS0, configMAX_SYSCALL_INTERRUPT_PRIORITY);
			IntEnable(INT_ADC0SS0);
			ROM_IntEnable(INT_GPIOB);
			if(parametro.din.PB0) {
				GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_0);
			} else {
				GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_0);
			}

			if(parametro.din.PB1) {
				GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_1);
			} else {
				GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_1);
			}

			if(parametro.din.PB2) {
				GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2);
			} else {
				GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_2);
			}

			if(parametro.din.PB3) {
				GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_3);
			} else {
				GPIOIntDisable(GPIO_PORTB_BASE, GPIO_PIN_3);
			}

			// Configurar los comparadores
			ADCComparatorConfigure( ADC0_BASE, 0, ADC_COMP_INT_NONE );
			ADCComparatorConfigure( ADC0_BASE, 1, ADC_COMP_INT_HIGH_ONCE|ADC_COMP_INT_LOW_ONCE );
			ADCComparatorConfigure( ADC0_BASE, 2, ADC_COMP_INT_HIGH_ONCE|ADC_COMP_INT_LOW_ONCE );
			ADCComparatorConfigure( ADC0_BASE, 3, ADC_COMP_INT_HIGH_ONCE|ADC_COMP_INT_LOW_ONCE );
			ADCComparatorConfigure( ADC0_BASE, 4, ADC_COMP_INT_HIGH_ONCE|ADC_COMP_INT_LOW_ONCE );

			//ADCComparatorRegionSet( ADC0_BASE, 0, 0, 0 );
			ADCComparatorRegionSet( ADC0_BASE, 1, parametro.level.L0, parametro.level.L0);
			ADCComparatorRegionSet( ADC0_BASE, 2, parametro.level.L1, parametro.level.L1 );
			ADCComparatorRegionSet( ADC0_BASE, 3, parametro.level.L2, parametro.level.L2 );
			ADCComparatorRegionSet( ADC0_BASE, 4, parametro.level.L3, parametro.level.L3 );

			ADCComparatorReset(ADC0_BASE, 0, true, true);
			ADCComparatorReset(ADC0_BASE, 1, true, true);
			ADCComparatorReset(ADC0_BASE, 2, true, true);
			ADCComparatorReset(ADC0_BASE, 3, true, true);
			ADCComparatorReset(ADC0_BASE, 4, true, true);
			ADCComparatorIntClear(ADC0_BASE, 31);
			ADCComparatorIntEnable(ADC0_BASE, 0);
		} else {
			IntDisable(INT_ADC0SS0);
			ROM_IntDisable(INT_GPIOB);
			TimerDisable(TIMER4_BASE, TIMER_A);
		}
	} else {
		return PROT_ERROR_INCORRECT_PARAM_SIZE;
	}

	return 0;
}

int32_t ComandoOscFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_OSC parametro;
	uint32_t ui32Period;

	if(check_and_extract_command_param(param, param_size, sizeof(parametro), &parametro) > 0) {
		if(parametro.osc.on) {
			ui32Period = (uint32_t)(SysCtlClockGet()/(parametro.osc.frecuencia));
			TimerLoadSet(TIMER2_BASE, TIMER_A,ui32Period );
			TimerEnable(TIMER2_BASE, TIMER_A);
		} else {
			TimerDisable(TIMER2_BASE, TIMER_A);
		}
	} else {
		return PROT_ERROR_INCORRECT_PARAM_SIZE;
	}

	return 0;
}

int32_t ComandoTimeFun(uint32_t param_size, void *param)
{
	PARAM_COMANDO_TIME parametro;
	uint32_t ui32Period;
	int32_t numdatos;
	if(check_and_extract_command_param(param, param_size, sizeof(parametro), &parametro) > 0) {
		if(parametro.recibir) { // Si queremos recibir hora lo decimos

			parametro.time.min = min;
			parametro.time.sec = sec;
			parametro.time.hora = hora+1;
			numdatos=create_frame(frame, COMANDO_TIME, &parametro, sizeof(parametro), MAX_FRAME_SIZE);
			if (numdatos>=0)
			{
				send_frame(frame,numdatos);
			}

			return numdatos;
		} else { // En caso contrario, la petición es el envío de hora
			TimerDisable(TIMER4_BASE, TIMER_B);
			hora = parametro.time.hora;
			min = parametro.time.min;
			sec = parametro.time.sec;

			ui32Period = SysCtlClockGet(); // 1 s
			// Carga la cuenta en el Timer4A
			TimerLoadSet(TIMER5_BASE, TIMER_A, ui32Period -1);
			// Empieza a funcionar
			TimerEnable(TIMER5_BASE, TIMER_A);
			return 0;
		}
	} else {
		return PROT_ERROR_INCORRECT_PARAM_SIZE;
	}
}

/* Array que contiene las funciones que se van a ejecutar en respuesta a cada comando */
static const remote_fun remote_fun_array[]={
		ComandoRechazadoFun, /* Responde al paquete comando rechazado */
		ComandoPingFun, /* Responde al comando ping */
		ComandoLedsFun, /* Responde al comando LEDS */
		ComandoRGBFun, /* Responde al comando RGB */
		ComandoBrilloLedsFun, /* Responde al comando Brillo */
		ComandoSwitchFun, /* Responde al comando Switch */
		ComandoModeFun, /* Responde al comando Modo */
		ComandoButtonsIntFun, /* Responde al comando ButtonsInt */
		ComandoADCFun, /* SEMANA2*/
		ComandoOscFun,
		ComandoNoImplementadoFun,
		ComandoTimeFun,

};

// Codigo para procesar los comandos recibidos a traves del canal USB del micro ("conector lateral")

//Esta tarea decodifica los comandos y ejecuta la función que corresponda a cada uno de ellos (por posicion)
//También gestiona posibles errores en la comunicacion
static portTASK_FUNCTION( CommandProcessingTask, pvParameters ){

	//Frame es global en este fichero, se reutiliza en las funciones que envian respuestas ---> CUIDADO!!!

	int32_t numdatos;
	uint8_t command;
	void *ptrtoparam;

	/* The parameters are not used. (elimina el warning)*/
	( void ) pvParameters;

	for(;;)
	{
		numdatos=receive_frame(frame,MAX_FRAME_SIZE);
		if (numdatos>0)
		{	//Si no hay error, proceso la trama que ha llegado.
			numdatos=destuff_and_check_checksum(frame,numdatos);
			if (numdatos<0)
			{
				//Error de checksum (PROT_ERROR_BAD_CHECKSUM), ignorar el paquete
				gRemoteProtocolErrors++;
				// Procesamiento del error (TODO, POR HACER!!)
			}
			else
			{
				//El paquete esta bien, luego procedo a tratarlo.
				command=decode_command_type(frame);
				numdatos=get_command_param_pointer(frame,numdatos,&ptrtoparam);

				if (command<(sizeof(remote_fun_array)/sizeof(remote_fun)))
				{
					switch(remote_fun_array[command](numdatos,ptrtoparam))
					{
						case COMANDO_PING :

							ComandoPingFun(0, NULL);

						break;
						case COMANDO_LEDS:
						{
							ComandoLedsFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						case COMANDO_RGB:
						{
							ComandoRGBFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						case COMANDO_BRILLO:
						{
							ComandoBrilloLedsFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						case COMANDO_SWITCH:
						{
							ComandoSwitchFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						case COMANDO_MODE:
						{
							ComandoModeFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						case COMANDO_BUTTONSINT:
						{
							ComandoButtonsIntFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						case COMANDO_ADC:
						{
							ComandoADCFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						case COMANDO_OSC:
						{
							ComandoOscFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						case COMANDO_TIME:
						{
							ComandoTimeFun(numdatos-CHECKSUM_SIZE-COMMAND_SIZE, frame+COMMAND_SIZE);
						}
						break;
						//La funcion puede devolver códigos de error.
					    //Se procesarían a continuación
						case PROT_ERROR_NOMEM:
						{
							PARAM_COMANDO_ERROR parametro;

							parametro.error.tipo = PROT_ERROR_NOMEM;
							//El comando es erróneo, envíamos tipo de error
							numdatos=create_frame(frame,COMANDO_ERROR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
							if (numdatos>=0)
							{
									send_frame(frame,numdatos);
							}

						}
						break;
						case PROT_ERROR_STUFFED_FRAME_TOO_LONG:
						{
							PARAM_COMANDO_ERROR parametro;

							parametro.error.tipo = PROT_ERROR_STUFFED_FRAME_TOO_LONG;
							//El comando es erróneo, envíamos tipo de error
							numdatos=create_frame(frame,COMANDO_ERROR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
							if (numdatos>=0)
							{
									send_frame(frame,numdatos);
							}
						}
						break;
						case PROT_ERROR_COMMAND_TOO_LONG:
						{
							PARAM_COMANDO_ERROR parametro;

							parametro.error.tipo = PROT_ERROR_COMMAND_TOO_LONG;
							//El comando es erróneo, envíamos tipo de error
							numdatos=create_frame(frame,COMANDO_ERROR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
							if (numdatos>=0)
							{
									send_frame(frame,numdatos);
							}
						}
						break;
						case PROT_ERROR_INCORRECT_PARAM_SIZE:
						{
							PARAM_COMANDO_ERROR parametro;

							parametro.error.tipo = PROT_ERROR_INCORRECT_PARAM_SIZE;
							//El comando es erróneo, envíamos tipo de error
							numdatos=create_frame(frame,COMANDO_ERROR,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
							if (numdatos>=0)
							{
									send_frame(frame,numdatos);
							}
						}
						break;
						case PROT_ERROR_UNIMPLEMENTED_COMMAND:
						{
							PARAM_COMANDO_RECHAZADO parametro;

							parametro.command=command;
							//El comando esta bien pero no esta implementado
							numdatos=create_frame(frame,COMANDO_RECHAZADO,&parametro,sizeof(parametro),MAX_FRAME_SIZE);
							if (numdatos>=0)
							{
									send_frame(frame,numdatos);
							}
						}
						break;
						//AÑadir casos de error aqui...
						default:
							/* No hacer nada */
							break;
					}
				}
				else
				{
						/* El comando no es reconocido por el microcontrolador */
						ComandoNoImplementadoFun(numdatos,ptrtoparam);
						gRemoteProtocolErrors++;
				}
			}
		}
		else
		{ // if (numdatos >0)
				//Error de recepcion de trama(PROT_ERROR_RX_FRAME_TOO_LONG), ignorar el paquete
				gRemoteProtocolErrors++;
				// Procesamiento del error (TODO)
		}
	}
}

//SEMANA2: añadida para facilitar el envio de datos a diversas tareas. IMPORTANTE!!! Leer los comentarios que hay abajo
//Ojo!! Frame es global (para ahorrar memoria de pila en las tareas) --> Se deben tomar precauciones al usar esta función en varias tareas
//IDEM en lo que respecta al envio por el puerto serie desde varias tareas....
//Estas precauciones no se han tomado en este codigo de partida, pero al realizar la practica se deberian tener en cuenta....
int32_t RemoteSendCommand(uint8_t comando,void *parameter,int32_t paramsize)
{
	int32_t numdatos;

	numdatos=create_frame(frame,comando,parameter,paramsize,MAX_FRAME_SIZE);
	if (numdatos>=0)
	{
		send_frame(frame,numdatos);
	}

	return numdatos;
}

//Inicializa la tarea que recibe comandos (se debe llamar desde main())
void RemoteInit(void)
{
	//
	// Crea la tarea que gestiona los comandos USB (definidos en CommandProcessingTask)
	//
	if(xTaskCreate(CommandProcessingTask, (portCHAR *)"usbser",REMOTE_TASK_STACK, NULL, REMOTE_TASK_PRIORITY, NULL) != pdTRUE)
	{
		while(1);
	}

}
