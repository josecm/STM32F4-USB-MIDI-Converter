#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* CUBEMX */
extern USBD_CDC_ItfTypeDef  USBD_Interface_fops_FS;
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
	 
/* JM */	 
extern volatile uint8_t USBD_Ready;

#ifdef __cplusplus
}
#endif
  
#endif /* __USBD_CDC_IF_H */
