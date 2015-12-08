#ifndef __MIDI_IO_H__
#define __MIDI_IO_H__

#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "FreeRtos.h"
#include "semphr.h"

typedef union { 
	
	struct {

		uint8_t midi_usb_header;
		uint8_t midi_status;
		uint8_t midi_data_1;
		uint8_t midi_data_2;
		
	} strct;

	uint8_t arr[4];
		
} MIDI_Packet;

#define MIDI_UART_IN_SIZE 	256
#define MIDI_UART_OUT_SIZE 	256
#define MIDI_USB_IN_SIZE		256

extern volatile uint32_t midi_uart_in_start;
extern volatile uint32_t midi_uart_in_end;
extern SemaphoreHandle_t midi_uart_in_sem;
extern MIDI_Packet midi_uart_in_buffer[MIDI_UART_IN_SIZE];

extern volatile uint32_t midi_uart_out_start;
extern volatile uint32_t midi_uart_out_end;
extern MIDI_Packet midi_uart_out_buffer[MIDI_UART_OUT_SIZE];
extern MIDI_Packet midi_usb_in_temp;
extern SemaphoreHandle_t midi_uart_out_empty_sem;

extern MIDI_Packet midi_usb_in_buffer[MIDI_USB_IN_SIZE];

static void MX_USART2_UART_Init(void);
void MIDI_UART_Init(void);

uint8_t MIDI_GetNoParam(uint8_t status);
uint8_t MIDI_GetUSBParam(uint8_t status);
#define MIDI_IsSysExMessage(status)  ((status) == 0xF7 || (status) == 0xF0)

MIDI_Packet GetMidiMessageUART(void);
MIDI_Packet GetMidiMessageUSB(void);

void SendMidiMessageUART(MIDI_Packet);
#define SendMidiMessageUSB(message) (CDC_Transmit_FS( (message).arr,  4))

void MIDI_UART_Receive_Callback(void);  
void MIDI_USB_Receive_Callback(uint8_t *buf, uint32_t len);
void MIDI_UART_Send_Callback(void);  

#endif
