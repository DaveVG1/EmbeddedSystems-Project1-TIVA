/* protocol.h
 *
 * Este fichero define las funciones para implementar la comunicacion mediante
 * el protocolo propuesto
 *
 * Los mensajes que se envian por el perfil USB constan de un byte de INICIO,
 * un byte de comando, un campo de datos opcional de longitud variable que depende del comando,
 * un CRC de 2 bytes y un byte de FIN.
 *
 */


#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include<stdlib.h>
#include<string.h>
#include<FreeRTOS.h>

//Caracteres especiales
#define START_FRAME_CHAR 0xFC
#define STOP_FRAME_CHAR 0xFD
#define ESCAPE_CHAR 0xFE
#define STUFFING_MASK 0x20

//Tipos de los campos
#define CHEKSUM_TYPE uint16_t
#define COMMAND_TYPE uint8_t


#define CHECKSUM_SIZE (sizeof(CHEKSUM_TYPE))
#define COMMAND_SIZE (sizeof(COMMAND_TYPE))
#define START_SIZE (1)
#define END_SIZE (1)

#define MINIMUN_FRAME_SIZE (START_SIZE+COMMAND_SIZE+CHECKSUM_SIZE+END_SIZE)

#define MAX_DATA_SIZE (32)
#define MAX_FRAME_SIZE (2*(MAX_DATA_SIZE))

//Codigos de Error del protocolo
#define PROT_ERROR_BAD_CHECKSUM (-1)
#define PROT_ERROR_RX_FRAME_TOO_LONG (-2)
#define PROT_ERROR_NOMEM (-3)
#define PROT_ERROR_STUFFED_FRAME_TOO_LONG (-4)
#define PROT_ERROR_COMMAND_TOO_LONG (-5)
#define PROT_ERROR_INCORRECT_PARAM_SIZE (-6)
#define PROT_ERROR_BAD_SIZE (-7)
#define PROT_ERROR_UNIMPLEMENTED_COMMAND (-7)


//Codigos de los comandos. EL estudiante deberá definir los códigos para los comandos que vaya
// a crear y usar. Estos deberan ser compatibles con los usados en la parte Qt
typedef enum {
	COMANDO_RECHAZADO,
	COMANDO_PING,
	COMANDO_LEDS,
	COMANDO_RGB,
	COMANDO_BRILLO,
	COMANDO_SWITCH,
	COMANDO_MODE,
	COMANDO_BUTTONSINT,
	COMANDO_ADC,
	COMANDO_OSC,
	COMANDO_RECIBIR_OSCILOSCOPIO,
	COMANDO_TIME,
	COMANDO_ERROR
	//etc, etc...
} commandTypes;

//Estructuras relacionadas con los parametros de los comandos. El estuadiante debera crear las
// estructuras adecuadas a los comandos usados, y asegurarse de su compatibilidad con el extremo Qt
//#pragma pack(1)	//Con esto consigo que el alineamiento de las estructuras en memoria del PC (32 bits) no tenga relleno.
#define PACKED __attribute__ ((packed))

typedef struct {
	uint8_t command;
} PACKED PARAM_COMANDO_RECHAZADO;

typedef union{
	struct {
		 uint8_t red;
		 uint8_t green;
		 uint8_t blue;
	} PACKED leds;
    uint8_t valor;
} PACKED PARAM_COMANDO_LEDS;

//Aqui puedo a�adir comandos....

typedef union{
	struct {
		 uint8_t red;
		 uint8_t green;
		 uint8_t blue;
	} PACKED rgb;
    uint8_t valor;
} PACKED PARAM_COMANDO_RGB;

typedef union{
    struct {
		uint8_t SW1:1;
		uint8_t SW2:1;
    } PACKED switches;
    uint8_t ui8Valor;
} PACKED PARAM_COMANDO_SWITCHES;

typedef struct {
    float rIntensity;
} PACKED PARAM_COMANDO_BRILLO;

typedef struct {
	uint8_t mode:1;
} PACKED PARAM_COMANDO_MODE;

typedef union{
    struct {
		uint8_t OK:1;
    } PACKED buttonsint;
    uint8_t ui8Valor;
} PACKED PARAM_COMANDO_BUTTONSINT;

typedef struct
{
	uint8_t ok:1;
    struct {
        uint8_t AIN0:1;
        uint8_t AIN1:1;
        uint8_t AIN2:1;
        uint8_t AIN3:1;
    } PACKED anin;
    struct {
        uint32_t L0;
        uint32_t L1;
        uint32_t L2;
        uint32_t L3;
    } PACKED level;
    struct {
        uint8_t PB0:1;
        uint8_t PB1:1;
        uint8_t PB2:1;
        uint8_t PB3:1;
    } PACKED din;
} PACKED PARAM_COMANDO_ADC;

typedef union{
    struct {
        uint32_t frecuencia;
        int on;
    } PACKED osc;
} PACKED PARAM_COMANDO_OSC;

typedef union{
	 struct {
		   uint8_t valor0[10];
		   uint8_t valor1[10];
		   uint8_t valor2[10];
		   uint8_t valor3[10];
	 }PACKED valores;
} PACKED PARAM_COMANDO_RECIBIR_OSCILOSCOPIO;

typedef union{
	uint8_t recibir:1;
	struct {
		uint32_t hora;
		uint32_t min;
		uint32_t sec;
	} PACKED time;
} PACKED PARAM_COMANDO_TIME;

typedef union{
    struct {
    	int8_t tipo;
    } PACKED error;
} PACKED PARAM_COMANDO_ERROR;

//#pragma pack()	//...Pero solo para los comandos que voy a intercambiar, no para el resto.

//Funciones que obtienen campos del paquete
uint8_t decode_command_type(uint8_t * buffer);
int32_t check_and_extract_command_param(void *ptrtoparam, int32_t param_size, uint32_t payload,void *param);
int32_t get_command_param_pointer(uint8_t * buffer, int32_t frame_size, void **campo);

//Funciones de la libreria
int32_t create_frame(uint8_t *frame, uint8_t command_type, void * param, int32_t param_size, int32_t max_size);
int32_t send_frame(uint8_t *frame, int32_t FrameSize);
int32_t receive_frame(uint8_t *frame, int32_t maxFrameSize);
int32_t destuff_and_check_checksum(uint8_t *frame, int32_t max_size);




#endif
