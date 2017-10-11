/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "synta.h"
#ifdef FOCUSER_ENABLE
#include "focuser.h"
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define FW_VER 0x0001

#ifdef DEBUG
#if defined ( __CC_ARM   )
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
#endif

/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
#ifdef MOUNT_ENABLE
volatile uint32_t packet_receive = 1;
#endif
#ifdef FOCUSER_ENABLE
volatile uint32_t f_packet_receive = 1;
uint8_t f_buf[30], f_idx;
__IO uint8_t f_cmd_buf[BUF_SIZE];
#endif
volatile uint32_t packet_sent = 1;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * Function Name  : PUTCHAR_PROTOTYPE
 * Description    : Retargets the C library DEBUG_PRINTF function to the USART.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
#ifdef DEBUG
#if defined ( __CC_ARM   )
PUTCHAR_PROTOTYPE {
    /* Write a character to the USART */
    USART_SendData(USART1, (u8) ch);

    /* Loop until the end of transmission */
    while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) {
    }

    return ch;
}
#endif
#endif

/*******************************************************************************
 * Function Name  : main.
 * Descriptioan    : Main routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
int main(void)
{
    uint32_t i;
#ifdef FOCUSER_ENABLE
#endif

    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
#ifdef DEBUG
    USART_Config_Default();
#endif
#ifdef MOUNT_ENABLE
    ra_motor_enable(0);
    dec_motor_enable(0);
    tim_init();
#endif
#ifdef FOCUSER_ENABLE
    focuser_init();
#endif

#ifdef MOUNT_ENABLE
    init_axis_data();
    //point to NCP, 0x800000 is the center
    axis_data[RA].position = 0x800000;
    axis_data[DEC].position = 0x800000 + axis_data[DEC].steps_per_axis / 4;
    calc_guiding_speed();
#endif

    DEBUG_PRINTF("\n\rEQMAX START\n\r");

    while(1) {
        if(bDeviceState == CONFIGURED) {
#ifdef MOUNT_ENABLE
            if(packet_receive) {
                packet_receive = 0;

                if (cmd_buf[1] == AXIS_RA || cmd_buf[1] == AXIS_DEC) {
                    axis = cmd_buf[1] - '1';

                    response_data = 0;
                    switch  (cmd_buf[0]) {
                        case 'e' :
                            response_data = FIRMWARE_VERSION;
                            axis_data[RA].enable();
                            axis_data[DEC].enable();
                            break;
                        case 'a' :
                            response_data = axis_data[axis].steps_per_axis;
                            break;
                        case 'b' :
                            response_data = axis_data[axis].sidereal_step_rate;
                            break;
                        case 'g' :
                            response_data = axis_data[axis].high_speed_multiplier;
                            break;
                        case 's' :
                            response_data = axis_data[axis].steps_per_worm;
                            break;
                        case 'f' :
                            response_data = axis_data[axis].status;
                            break;
                        case 'j' :
                            response_data = axis_data[axis].position;
                            break;
                        case 'K' :
                            axis_data[axis].stop();
                            break;
                        case 'L' :
                            axis_data[axis].emergency_stop();
                            break;
                        case 'G' :
                            axis_data[axis].mode = cmd_buf[2] - '0';
                            axis_data[axis].direction = cmd_buf[3] - '0';
                            /* set dir flag */
                            if (axis_data[axis].direction == 0)
                                axis_data[axis].status &= ~AXIS_DIR;
                            else
                                axis_data[axis].status |= AXIS_DIR;

                            break;
                        case 'H' :
                            axis_data[axis].goto_position = 
                                char_to_nibble(cmd_buf[3]) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[2]))) << 4) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[5]))) << 8) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[4]))) << 12) +
                                (((uint32_t)(char_to_nibble(cmd_buf[7]))) << 16) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[6]))) << 20) ; 
                            DEBUG_PRINTF("\n\rgoto_position=%ld\n\r", axis_data[axis].goto_position);
                            axis_data[axis].status |= AXIS_GOTO;
                            break;
                        case 'I' :
                            axis_data[axis].speed = 
                                char_to_nibble(cmd_buf[3]) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[2]))) << 4) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[5]))) << 8) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[4]))) << 12) +
                                (((uint32_t)(char_to_nibble(cmd_buf[7]))) << 16) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[6]))) << 20) ; 
                            /*if (axis_data[axis].status & AXIS_MOVING)
                                axis_data[axis].update_speed();*/
                            break;
                        case 'E' :
                            axis_data[axis].position = 
                                char_to_nibble(cmd_buf[3]) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[2]))) << 4) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[5]))) << 8) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[4]))) << 12) +
                                (((uint32_t)(char_to_nibble(cmd_buf[7]))) << 16) + 
                                (((uint32_t)(char_to_nibble(cmd_buf[6]))) << 20) ; 
                            break;
                        case 'F' :
                            axis_data[axis].enable();
                            break;
                        case 'J' :
                            axis_data[axis].action();
                            break;
                        case 'W' :
                            for (i = 0; i < sizeof(save_axis_data); i++)
                                ((uint8_t*)&response[1])[i] = ((uint8_t*)save_axis_data)[i];
                            break;
                        case 'V' :
                            for (i = 0; i < sizeof(save_axis_data); i++)
                                ((uint8_t*)save_axis_data)[i] = ((uint8_t*)&cmd_buf[2])[i];
                            FLASH_Unlock();
                            FLASH_ClearFlag(FLASH_FLAG_EOP|FLASH_FLAG_PGERR|FLASH_FLAG_WRPRTERR);
                            FLASH_ErasePage(ADDR_DATA);
                            for (i = 0; i < sizeof(save_axis_data) / 2; i++)
                                FLASH_ProgramHalfWord(ADDR_DATA + i * 2, ((uint16_t*)save_axis_data)[i]);
                            FLASH_Lock();
                            axis_data[RA].motor_mode = save_axis_data[RA].motor_mode; 
                            axis_data[RA].sidereal_scaler = save_axis_data[RA].sidereal_scaler; 
                            axis_data[RA].steps_per_axis = save_axis_data[RA].steps_per_axis; 
                            axis_data[RA].steps_per_worm = save_axis_data[RA].steps_per_worm; 
                            axis_data[RA].sidereal_step_rate = save_axis_data[RA].sidereal_step_rate; 
                            axis_data[RA].high_speed_multiplier = save_axis_data[RA].high_speed_multiplier; 
                            axis_data[RA].max_speed = save_axis_data[RA].max_speed; 
                            axis_data[RA].guiding_speed_factor = save_axis_data[RA].guiding_speed_factor; 
                            axis_data[RA].revert = save_axis_data[RA].revert; 
                            axis_data[DEC].motor_mode = save_axis_data[DEC].motor_mode; 
                            axis_data[DEC].sidereal_scaler = save_axis_data[DEC].sidereal_scaler; 
                            axis_data[DEC].steps_per_axis = save_axis_data[DEC].steps_per_axis; 
                            axis_data[DEC].steps_per_worm = save_axis_data[DEC].steps_per_worm; 
                            axis_data[DEC].sidereal_step_rate = save_axis_data[DEC].sidereal_step_rate; 
                            axis_data[DEC].high_speed_multiplier = save_axis_data[DEC].high_speed_multiplier; 
                            axis_data[DEC].max_speed = save_axis_data[DEC].max_speed; 
                            axis_data[DEC].guiding_speed_factor = save_axis_data[DEC].guiding_speed_factor; 
                            axis_data[DEC].revert = save_axis_data[DEC].revert; 

                            response_data = 1;
                            calc_guiding_speed();

                            DEBUG_PRINTF("\n\r motor_mode = %ul/%ul", save_axis_data[0].motor_mode, save_axis_data[1].motor_mode);
                            DEBUG_PRINTF("\n\r sidereal_scaler = %u/%u", save_axis_data[0].sidereal_scaler, save_axis_data[1].sidereal_scaler);
                            DEBUG_PRINTF("\n\r steps_per_axis = %lu/%lu", save_axis_data[0].steps_per_axis, save_axis_data[1].steps_per_axis);
                            DEBUG_PRINTF("\n\r steps_per_worm = %lu/%lu", save_axis_data[0].steps_per_worm, save_axis_data[1].steps_per_worm);
                            DEBUG_PRINTF("\n\r sidereal_step_rate = %lu/%lu", save_axis_data[0].sidereal_step_rate, save_axis_data[1].sidereal_step_rate);
                            DEBUG_PRINTF("\n\r high_speed_multiplier = %lu/%lu", save_axis_data[0].high_speed_multiplier, save_axis_data[1].high_speed_multiplier);
                            DEBUG_PRINTF("\n\r max_speed = %lu/%lu", save_axis_data[0].max_speed, save_axis_data[1].max_speed);
                            DEBUG_PRINTF("\n\r guiding_speed_factor = %f/%f", save_axis_data[0].guiding_speed_factor, save_axis_data[1].guiding_speed_factor);

                            break;
                        case 'U' :
                            axis_data[axis].position = 0;
                            axis_data[axis].enable();
                            axis_data[axis].mode = MODE_SLEW;
                            axis_data[axis].direction = 0;
                            axis_data[axis].status &= ~AXIS_DIR;
                            axis_data[axis].speed = axis_data[axis].sidereal_scaler / 24;
                            axis_data[axis].action();
                            break;
                        case 'T' :
                            axis_data[axis].stop();
                            while(!(axis_data[axis].status & AXIS_STOPED));
                            for (i = 0; i < 4; i++)
                                ((uint8_t*)&response[1])[i] = ((uint8_t*)&axis_data[axis].position)[i];
                            break;
                        case 'S' :
                            axis_data[axis].enable();
                            axis_data[axis].mode = MODE_SLEW;
                            axis_data[axis].direction = 0;
                            axis_data[axis].status &= ~AXIS_DIR;
                            axis_data[axis].speed = axis_data[axis].sidereal_scaler / 24;
                            axis_data[axis].action();
                            break;
                        case 'R' :
                            ((uint8_t*)&response[1])[0] = FW_TYPE >> 8;
                            ((uint8_t*)&response[1])[1] = FW_TYPE & 0x00FF;
                            ((uint8_t*)&response[1])[2] = FW_VER >> 8;
                            ((uint8_t*)&response[1])[3] = FW_VER & 0x00FF;
                            for (i = 4; i < 16; i++)
                                ((uint8_t*)&response[1])[i] = 0;
                            break;
                        default : 
                            break;
                    }

                    prepare_response(response, cmd_buf[0], response_data);
                    DEBUG_PRINTF("\n\rRX:%s | TX:%s", (char *)cmd_buf, response);
                    CDC_Send_DATA((uint8_t *)response, get_response_len(cmd_buf[0]));
                }
            }
#endif

#ifdef FOCUSER_ENABLE
            if (f_packet_receive > 0) {
                //f_packet_receive = 0;
                f_idx = 0;
                f_packet_receive = 0;
            }
#endif
        }
    }
}
