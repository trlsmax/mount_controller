/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "synta.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#ifdef MOUNT_ENABLE
__IO uint8_t ra_stop_changing, dec_stop_changing;
__IO uint8_t ra_idx, dec_idx;
__IO uint8_t ra_need_to_stop, dec_need_to_stop;
__IO uint32_t ra_steps, dec_steps;
__IO uint8_t ra_guiding = ST4_RA_NONE, dec_guiding = ST4_DEC_NONE;
extern volatile struct tim_control_ ra_tim_control[MAX_STEP_NUM + 1];
extern volatile struct tim_control_ dec_tim_control[MAX_STEP_NUM + 1];
extern volatile struct tim_control_ ra_guide_tim[2], dec_guide_tim;
extern TIM_TypeDef *axis_tim[2];
#endif
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                         */
/******************************************************************************/

/*******************************************************************************
 * Function Name  : NMI_Handler
 * Description    : This function handles NMI exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
 * Function Name  : HardFault_Handler
 * Description    : This function handles Hard Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    while(1) {
    }
}

/*******************************************************************************
 * Function Name  : MemManage_Handler
 * Description    : This function handles Memory Manage exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while(1) {
    }
}

/*******************************************************************************
 * Function Name  : BusFault_Handler
 * Description    : This function handles Bus Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while(1) {
    }
}

/*******************************************************************************
 * Function Name  : UsageFault_Handler
 * Description    : This function handles Usage Fault exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while(1) {
    }
}

/*******************************************************************************
 * Function Name  : SVC_Handler
 * Description    : This function handles SVCall exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
 * Function Name  : DebugMon_Handler
 * Description    : This function handles Debug Monitor exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void DebugMon_Handler(void)
{
}

/*******************************************************************************
 * Function Name  : PendSV_Handler
 * Description    : This function handles PendSVC exception.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
 * Function Name  : SysTick_Handler
 * Description    : This function handles SysTick Handler.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void SysTick_Handler(void)
{
}

/*******************************************************************************
 * Function Name  : USB_IRQHandler
 * Description    : This function handles USB Low Priority interrupts
 *                  requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    USB_Istr();
}

/*******************************************************************************
 * Function Name  : USB_FS_WKUP_IRQHandler
 * Description    : This function handles USB WakeUp interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/

void USBWakeUp_IRQHandler(void)
{
    EXTI_ClearITPendingBit(EXTI_Line18);
}

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                            */
/******************************************************************************/

/*******************************************************************************
 * Function Name  : PPP_IRQHandler
 * Description    : This function handles PPP interrupt request.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
/*void PPP_IRQHandler(void)
  {
  }*/

#ifdef MOUNT_ENABLE
void TIM1_UP_IRQHandler(void)
{
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

    axis_data[RA].direction == 0 ? axis_data[RA].position++ : axis_data[RA].position--;

    if (ra_guiding == ST4_RA_NONE) {
        if(ra_need_to_stop) {
            ra_need_to_stop = FALSE;
            ra_steps = 0;
            ra_idx = FIND_STOP_INDEX(ra_step_num, ra_idx);
            ra_stop_changing = FALSE;
        }

        if(!ra_stop_changing && ++ra_steps > ra_tim_control[tim_idx[ra_idx]].steps) {
            ra_steps = 0;
            if(ra_tim_control[tim_idx[ra_idx]].flag & AXIS_DECELLERATE)
                ra_idx = FIND_STOP_INDEX(ra_step_num, ra_idx);
            else
                ra_idx++;

            TIM_Cmd(TIM1, DISABLE);
            if(tim_idx[ra_idx] != 0) {
                TIM1->PSC = ra_tim_control[tim_idx[ra_idx]].prescaler;
                TIM1->ARR = ra_tim_control[tim_idx[ra_idx]].period;
                TIM1->CCR1 = ra_tim_control[tim_idx[ra_idx]].ccr;
                //TIM1->CNT = 0;
                TIM_Cmd(TIM1, ENABLE);

                if(ra_tim_control[tim_idx[ra_idx]].flag & STOP_CHANGING)
                    ra_stop_changing = TRUE;

                if(ra_idx > ra_step_num)
                    axis_data[RA].status |= AXIS_STOPING;
            } else {
                axis_data[RA].status |= AXIS_STOPED;
                axis_data[RA].status &= ~AXIS_GOTO;
                ra_motor_enable(0);
                ra_step_num = 0;
            }
        }
    }
}

void TIM2_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        axis_data[DEC].direction == 0 ? axis_data[DEC].position++ : axis_data[DEC].position--;

        if (dec_guiding == ST4_DEC_NONE) {
            if(dec_need_to_stop) {
                dec_need_to_stop = FALSE;
                dec_steps = 0;
                dec_idx = FIND_STOP_INDEX(dec_step_num, dec_idx);
                dec_stop_changing = FALSE;
            }

            if(!dec_stop_changing && ++dec_steps > dec_tim_control[tim_idx[dec_idx]].steps) {
                dec_steps = 0;
                if(dec_tim_control[tim_idx[dec_idx]].flag & AXIS_DECELLERATE)
                    dec_idx = FIND_STOP_INDEX(dec_step_num, dec_idx);
                else
                    dec_idx++;

                TIM_Cmd(TIM2, DISABLE);
                if(tim_idx[dec_idx] != 0) {
                    TIM2->PSC = dec_tim_control[tim_idx[dec_idx]].prescaler;
                    TIM2->ARR = dec_tim_control[tim_idx[dec_idx]].period;
                    TIM2->CCR4 = dec_tim_control[tim_idx[dec_idx]].ccr;
                    //TIM2->CNT = 0;
                    TIM_Cmd(TIM2, ENABLE);

                    if(dec_tim_control[tim_idx[dec_idx]].flag & STOP_CHANGING)
                        dec_stop_changing = TRUE;

                    if(dec_idx > dec_step_num)
                        axis_data[DEC].status |= AXIS_STOPING;
                } else {
                    axis_data[DEC].status |= AXIS_STOPED;
                    axis_data[DEC].status &= ~AXIS_GOTO;
                    dec_motor_enable(0);
                    dec_step_num = 0;
                }
            }
        }
    }
}

/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
    /* ST4_RA+ */
    if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
        DEBUG_PRINTF("\n\rST4_RA+ ");
        if ((GPIOB->IDR & GPIO_Pin_6) != Bit_RESET) {
            DEBUG_PRINTF("\n\rRising edge");
            /* Rising Edge */
            if (axis_data[RA].status & AXIS_STOPED || ra_step_num == 2) {
                EXTI->IMR &= ~EXTI_Line9;
                TIM_Cmd(axis_tim[RA], DISABLE);
                ra_motor_enable(1);
                ra_step_num = 0;
                ra_motor_dir(0);
                axis_data[RA].direction = 0;
                axis_tim[RA]->PSC = ra_guide_tim[0].prescaler;
                axis_tim[RA]->ARR = ra_guide_tim[0].period;
                axis_tim[RA]->CCR1 = ra_guide_tim[0].ccr;
                axis_tim[RA]->CNT = 0;
                TIM_Cmd(axis_tim[RA], ENABLE);
                ra_guiding = ST4_RA_P;
                axis_data[RA].status &= ~(AXIS_STOPED | AXIS_STOPING);
            }
        } else {
            DEBUG_PRINTF("\n\rFalling edge");
            if (ra_guiding == ST4_RA_P) {
                EXTI->IMR |= EXTI_Line9;
                TIM_Cmd(axis_tim[RA], DISABLE);
                ra_guiding = ST4_RA_NONE;
                ra_motor_enable(0);
                axis_data[RA].status |= AXIS_STOPED;
            }
        }
        /* Clear the  EXTI line 6 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line6);
    }

    /* ST4_DEC+ */
    if(EXTI_GetITStatus(EXTI_Line7) != RESET) {
        DEBUG_PRINTF("\n\rST4_DEC+ ");
        if ((GPIOB->IDR & GPIO_Pin_7) != Bit_RESET) {
            DEBUG_PRINTF("\n\rRising edge");
            /* Rising Edge */
            if (axis_data[DEC].status & AXIS_STOPED || dec_step_num == 2) {
                EXTI->IMR &= ~EXTI_Line8;
                TIM_Cmd(axis_tim[DEC], DISABLE);
                dec_motor_enable(1);
                dec_step_num = 0;
                dec_motor_dir(0);
                axis_data[DEC].direction = 0;
                axis_tim[DEC]->PSC = dec_guide_tim.prescaler;
                axis_tim[DEC]->ARR = dec_guide_tim.period;
                axis_tim[DEC]->CCR1 = dec_guide_tim.ccr;
                axis_tim[DEC]->CNT = 0;
                TIM_Cmd(axis_tim[DEC], ENABLE);
                dec_guiding = ST4_DEC_P;
                axis_data[DEC].status &= ~(AXIS_STOPED | AXIS_STOPING);
            }
        } else {
            DEBUG_PRINTF("\n\rFalling edge");
            if (dec_guiding == ST4_DEC_P) {
                EXTI->IMR |= EXTI_Line8;
                TIM_Cmd(axis_tim[DEC], DISABLE);
                dec_guiding = ST4_DEC_NONE;
                dec_motor_enable(0);
                axis_data[DEC].status |= AXIS_STOPED;
            }
        }
        /* Clear the  EXTI line 7 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line7);
    }

    /* ST4_DEC- */
    if(EXTI_GetITStatus(EXTI_Line8) != RESET) {
        DEBUG_PRINTF("\n\rST4_DEC- ");
        if ((GPIOB->IDR & GPIO_Pin_8) != Bit_RESET) {
            DEBUG_PRINTF("\n\rRising edge");
            /* Rising Edge */
            if (axis_data[DEC].status & AXIS_STOPED || dec_step_num == 2) {
                EXTI->IMR &= ~EXTI_Line7;
                TIM_Cmd(axis_tim[DEC], DISABLE);
                dec_motor_enable(1);
                dec_step_num = 0;
                dec_motor_dir(1);
                axis_data[DEC].direction = 1;
                axis_tim[DEC]->PSC = dec_guide_tim.prescaler;
                axis_tim[DEC]->ARR = dec_guide_tim.period;
                axis_tim[DEC]->CCR1 = dec_guide_tim.ccr;
                axis_tim[DEC]->CNT = 0;
                TIM_Cmd(axis_tim[DEC], ENABLE);
                dec_guiding = ST4_DEC_M;
                axis_data[DEC].status &= ~(AXIS_STOPED | AXIS_STOPING);
            }
        } else {
            DEBUG_PRINTF("\n\rFalling edge");
            if (dec_guiding == ST4_DEC_M) {
                EXTI->IMR |= EXTI_Line7;
                TIM_Cmd(axis_tim[DEC], DISABLE);
                dec_guiding = ST4_DEC_NONE;
                dec_motor_enable(0);
                axis_data[DEC].status |= AXIS_STOPED;
            }
        }
        /* Clear the  EXTI line 8 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line8);
    }

    /* ST4_RA- */
    if(EXTI_GetITStatus(EXTI_Line9) != RESET) {
        DEBUG_PRINTF("\n\rST4_RA- ");
        if ((GPIOB->IDR & GPIO_Pin_9) != Bit_RESET) {
            DEBUG_PRINTF("\n\rRising edge");
            /* Rising Edge */
            if (axis_data[RA].status & AXIS_STOPED || ra_step_num == 2) {
                EXTI->IMR &= ~EXTI_Line6;
                TIM_Cmd(axis_tim[RA], DISABLE);
                ra_motor_enable(1);
                ra_step_num = 0;
                ra_motor_dir(0);
                axis_data[RA].direction = 1;
                axis_tim[RA]->PSC = ra_guide_tim[1].prescaler;
                axis_tim[RA]->ARR = ra_guide_tim[1].period;
                axis_tim[RA]->CCR1 = ra_guide_tim[1].ccr;
                axis_tim[RA]->CNT = 0;
                TIM_Cmd(axis_tim[RA], ENABLE);
                ra_guiding = ST4_RA_M;
                axis_data[RA].status &= ~(AXIS_STOPED | AXIS_STOPING);
            }
        } else {
            DEBUG_PRINTF("\n\rFalling edge");
            if (ra_guiding == ST4_RA_M) {
                EXTI->IMR |= EXTI_Line6;
                TIM_Cmd(axis_tim[RA], DISABLE);
                ra_guiding = ST4_RA_NONE;
                ra_motor_enable(0);
                axis_data[RA].status |= AXIS_STOPED;
            }
        }
        /* Clear the  EXTI line 9 pending bit */
        EXTI_ClearITPendingBit(EXTI_Line9);
    }
}
#endif
