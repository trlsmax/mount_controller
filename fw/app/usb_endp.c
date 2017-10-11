/* Includes ------------------------------------------------------------------*/
//#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "synta.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t packet_sent;
extern __IO uint32_t packet_receive;
extern __IO uint32_t f_packet_receive;
extern __IO uint8_t Receive_Buffer[64];
__IO uint8_t F_Receive_Buffer[64];
uint32_t Receive_length;
uint32_t F_Receive_length;

//extern __IO uint8_t cmd_buf[50];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/

void EP1_IN_Callback(void)
{
    packet_sent = 1;
}

#if defined(MOUNT_ENABLE) && defined(FOCUSER_ENABLE)
void EP4_IN_Callback(void)
{
    packet_sent = 1;
}
#endif

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void)
{
#ifdef MOUNT_ENABLE
    static int16_t len = 0;
    uint16_t i;
    static uint16_t state = 0, idx = 0;

    Receive_length = GetEPRxCount(ENDP3);
    PMAToUserBufferCopy((unsigned char*)Receive_Buffer, ENDP3_RXADDR, Receive_length);
    //Receive_length = USB_SIL_Read(EP3_OUT, (unsigned char*)Receive_Buffer);
#ifdef DEBUG
    Receive_Buffer[Receive_length] = '\0';
    DEBUG_PRINTF("\n\r[%lu]%s\n\r", Receive_length, Receive_Buffer);
#endif

    for(i = 0; i < Receive_length; i++) {
        if (state == 0) {
            if (Receive_Buffer[i] == ':')
                state = 1;
        } else if (state == 1) {
            if (vavid_cmd(Receive_Buffer[i])) {
                cmd_buf[idx++] = Receive_Buffer[i];
                len = get_cmddata_len(Receive_Buffer[i]) + 2;
                state = 2;
            } else {
                idx = 0;
                state = 0;
                len = 0;
            }
        } else if (state == 2) {
            if (idx < len)
                cmd_buf[idx++] = Receive_Buffer[i];
            else if (idx == len) {
                if (Receive_Buffer[i] == '\r')
                    packet_receive = Receive_length;

                idx = 0;
                state = 0;
                len = 0;
            }
        } else {
            idx = 0;
            state = 0;
            len = 0;
        }
    }

    /* Enable the receive of data on EP3 */
    SetEPRxValid(ENDP3);
#else
    uint16_t i;

    F_Receive_length = GetEPRxCount(ENDP3);
    PMAToUserBufferCopy((unsigned char*)F_Receive_Buffer, ENDP3_RXADDR, F_Receive_length);
#ifdef DEBUG
    F_Receive_Buffer[F_Receive_length] = '\0';
    DEBUG_PRINTF("\n\rEP3 got: [%d]%s", F_Receive_length, F_Receive_Buffer);
#endif

    for (i = 0; i < F_Receive_length; i++)
        f_cmd_buf[i] = F_Receive_Buffer[i];
    f_packet_receive = F_Receive_length;

    SetEPRxValid(ENDP3);
#endif
}

#if defined(MOUNT_ENABLE) && defined(FOCUSER_ENABLE)
void EP6_OUT_Callback(void)
{
    uint16_t i;

    F_Receive_length = GetEPRxCount(ENDP6);
    PMAToUserBufferCopy((unsigned char*)F_Receive_Buffer, ENDP6_RXADDR, F_Receive_length);
    //F_Receive_Buffer[F_Receive_length] = '\0';
    //DEBUG_PRINTF("\n\rEP6 got: [%d]%s", F_Receive_length, F_Receive_Buffer);

    for (i = 0; i < F_Receive_length; i++)
        f_cmd_buf[i] = F_Receive_Buffer[i];
    f_packet_receive = F_Receive_length;

    SetEPRxValid(ENDP6);

}
#endif
