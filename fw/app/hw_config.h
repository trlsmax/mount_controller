/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usb_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040
#define LED_ON                0xF0
#define LED_OFF               0xFF

/*Unique Devices IDs register set*/
#define         ID1          (0x1FFFF7E8)
#define         ID2          (0x1FFFF7EC)
#define         ID3          (0x1FFFF7F0)

/* Exported functions ------------------------------------------------------- */
void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void Get_SerialNum(void);
void LCD_Control(void);
uint32_t CDC_Send_DATA(uint8_t *ptrBuffer, uint8_t Send_length);
void USART_Config_Default(void);
void tim_init(void);
void ra_motor_enable(uint8_t enable);
void dec_motor_enable(uint8_t enable);
void ra_motor_dir(uint8_t dir);
void dec_motor_dir(uint8_t dir);
void ra_motor_mode(uint8_t mode);
void dec_motor_mode(uint8_t mode);
#if defined(MOUNT_ENABLE) && defined(FOCUSER_ENABLE)
uint32_t ep4_send(uint8_t *ptrBuffer, uint8_t Send_length);
#endif
/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
