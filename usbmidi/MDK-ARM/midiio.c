#include "midiio.h"

/* JM_MIDI  */

extern UART_HandleTypeDef huart2;

MIDI_Packet midi_uart_in_buffer[MIDI_UART_IN_SIZE];
MIDI_Packet midi_uart_out_buffer[MIDI_UART_OUT_SIZE];
MIDI_Packet midi_usb_in_buffer[MIDI_USB_IN_SIZE];

volatile uint32_t midi_uart_in_start = 0;
volatile uint32_t midi_uart_in_end = 0;
SemaphoreHandle_t midi_uart_in_sem;

volatile uint32_t midi_uart_out_start = 0;
volatile uint32_t midi_uart_out_end = 0;
SemaphoreHandle_t midi_uart_out_empty_sem;
SemaphoreHandle_t midi_uart_out_full_sem;

volatile uint32_t midi_usb_in_start = 0;
volatile uint32_t midi_usb_in_end = 0;
SemaphoreHandle_t midi_usb_empty_sem;
SemaphoreHandle_t midi_usb_full_sem;


uint8_t midi_uart_out_busy = 0;
uint8_t midi_uart_out_count = 0;
MIDI_Packet midi_uart_out_packet;

void MIDI_UART_Init(void){
	
	midi_uart_out_empty_sem = xSemaphoreCreateCounting(MIDI_UART_OUT_SIZE, MIDI_UART_OUT_SIZE);
	midi_uart_out_full_sem = xSemaphoreCreateCounting(MIDI_UART_OUT_SIZE, 0);
	
	midi_usb_empty_sem = xSemaphoreCreateCounting(MIDI_USB_IN_SIZE, MIDI_USB_IN_SIZE);
	midi_usb_full_sem =  xSemaphoreCreateCounting(MIDI_USB_IN_SIZE, 0);
	
	midi_uart_in_sem = xSemaphoreCreateCounting(MIDI_UART_IN_SIZE, 0);
	
	MX_USART2_UART_Init();	
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
		
}


void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 31250;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
	
	

}

uint8_t MIDI_GetNoParam(uint8_t status){
	
	status &= 0xF0;
	
	if(status == 0xF0){
		return 0;
	} else if(status == 0xD0){
		return 1;
	} else {
		return 2;
	}
	
}

uint8_t MIDI_GetUSBParam(uint8_t status){
	
	uint8_t temp = status & 0xF0;
	uint8_t temp2;
	
	if(temp >= 0x80 && temp <= 0xE0){	
		return (temp >> 4) | 0x10;
	} else if(temp == 0xF0){
		temp2 = status & 0x0F;	
		
		if(temp2 == 0x01 || temp2 == 0x03){
			return (0x20 >> 4) | 0x10;
		} else if (temp2 == 0x02){
			return (0x30 >> 4) | 0x10;
		} else if (temp2 >= 0x06){
			return (0x50 >> 4) | 0x10;
		} else {
			return 0;			
		}
	}
	
	return 0;
	
}


void MIDI_UART_Receive_Callback(){
	
	static MIDI_Packet midi_in;
	static uint8_t counter = 0;
	uint8_t byte = huart2.Instance->DR & 0xFF;
	
	if(counter == 0){
		
		counter = MIDI_GetNoParam(byte);
		
		midi_in.arr[1] = byte;
		midi_in.arr[0] = MIDI_GetUSBParam(byte);
		
		midi_in.arr[2]=  midi_in.arr [3] = 0;
		
	} else {
		
		if(counter == 2)
			midi_in.arr[2] = byte;
		else if(counter == 1)
			midi_in.arr[3] = byte;
		
		counter--;
	}
	
	if(counter == 0){
		
		if(midi_uart_in_start + MIDI_UART_IN_SIZE != midi_uart_in_end){
			midi_uart_in_buffer[midi_uart_in_end++ & (MIDI_UART_IN_SIZE-1)] = midi_in;
			xSemaphoreGiveFromISR(midi_uart_in_sem, NULL);
		}
		
	}
	
}



void MIDI_UART_Send_Callback(){
	
	
	if(midi_uart_out_count == 0){
		
		if(xSemaphoreTakeFromISR(midi_uart_out_full_sem, NULL)  == pdTRUE){
			
			midi_uart_out_packet = midi_uart_out_buffer[midi_uart_out_start++ & (MIDI_UART_OUT_SIZE - 1)];
			xSemaphoreGiveFromISR(midi_uart_out_empty_sem, NULL);
			midi_uart_out_count = MIDI_GetNoParam(midi_uart_out_packet.strct.midi_status);
			huart2.Instance->DR = midi_uart_out_packet.arr[1];
			
		} else {
			midi_uart_out_busy = 0;
		}
	} else if (midi_uart_out_count > 0){
	
		huart2.Instance->DR = midi_uart_out_packet.arr[2 + (midi_uart_out_count-- & 0x1)];

	}
	
	__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
}

void MIDI_USB_Receive_Callback(uint8_t *buf, uint32_t len){
	
	uint8_t count = len/4;
	
	for(int i = 0; i < count; i++){
		if (!MIDI_IsSysExMessage(buf[i*4+1])){
			if(xSemaphoreTakeFromISR(midi_usb_empty_sem, NULL)  == pdTRUE){
				midi_usb_in_buffer[midi_usb_in_end++ & (MIDI_USB_IN_SIZE -1)] = *((MIDI_Packet*) &(buf[i*4]));
				xSemaphoreGiveFromISR(midi_usb_full_sem, NULL);
			}
		}
	}
	
	USBD_CDC_ReceivePacket(&hUsbDeviceFS);
}


void SendMidiMessageUART(MIDI_Packet message){
	
	if(midi_uart_out_busy){
	
	 xSemaphoreTake(midi_uart_out_empty_sem, portMAX_DELAY);
		
		__HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);
		
		midi_uart_out_buffer[midi_uart_out_end++ & (MIDI_UART_OUT_SIZE -1)] = message;
		
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
	
		xSemaphoreGive(midi_uart_out_full_sem);
		//huart2.Instance->SR |= UART_FLAG_TC;
		
	} else {
		
		midi_uart_out_busy = 1;
		midi_uart_out_packet = message;
		midi_uart_out_count = MIDI_GetNoParam(message.strct.midi_status);
		huart2.Instance->DR = message.strct.midi_status;
		
	}
}


MIDI_Packet GetMidiMessageUART(void){
	
	MIDI_Packet temp;
	
	xSemaphoreTake(midi_uart_in_sem, portMAX_DELAY);
	
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_RXNE);
	
	temp =  midi_uart_in_buffer[midi_uart_in_start++ & (MIDI_UART_IN_SIZE-1)];
	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
	
	return temp;
}

MIDI_Packet GetMidiMessageUSB(void){
	
	MIDI_Packet temp;
	
	xSemaphoreTake(midi_usb_full_sem, portMAX_DELAY);
	//FALTAVA AQUI UMA MUTEX OU DESLIGAR AS INTERRUPÇOES DE USB RECEIVE
	temp = midi_usb_in_buffer[midi_usb_in_start++ & (MIDI_USB_IN_SIZE - 1)];
	//...
	xSemaphoreGive(midi_usb_empty_sem);
	return temp;
	
}
