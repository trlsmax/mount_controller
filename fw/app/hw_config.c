/* Includes ------------------------------------------------------------------*/

#include "stm32_it.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "synta.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;
extern __IO uint32_t packet_sent;
extern __IO uint8_t Send_Buffer[VIRTUAL_COM_PORT_DATA_SIZE] ;
extern __IO  uint32_t packet_receive;
extern __IO uint8_t Receive_length;

uint8_t Receive_Buffer[64];
uint32_t Send_length;
static void IntToUnicode(uint32_t value , uint8_t *pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;
uint16_t tmp_reg = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
#ifdef MOUNT_ENABLE
    NVIC_InitTypeDef NVIC_InitStructure;
#endif

    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32f10x_xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32f10x.c file
       */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    /* Enable USB_DISCONNECT GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    /* Configure USB pull-up pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure the EXTI line 18 connected internally to the USB IP */
    EXTI_ClearITPendingBit(EXTI_Line18);
    EXTI_InitStructure.EXTI_Line = EXTI_Line18;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

#ifdef MOUNT_ENABLE
    /* motor driver IO config */
    /* RA */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /* DEC */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ST4 PORT */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource7);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

    EXTI_InitStructure.EXTI_Line = EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | EXTI_Line9;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
    /* Select USBCLK source */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

    /* Enable the USB clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
    /* Set the device state to suspend */
    bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
    DEVICE_INFO *pInfo = &Device_Info;

    /* Set the device state to the correct state */
    if(pInfo->Current_Configuration != 0) {
        /* Device configured */
        bDeviceState = CONFIGURED;
    } else {
        bDeviceState = ATTACHED;
    }
    /*Enable SystemCoreClock*/
    SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 2 bit for pre-emption priority, 2 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    /* Enable the USB interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable the USB Wake-up interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config(FunctionalState NewState)
{
    if(NewState != DISABLE) {
        GPIO_ResetBits(GPIOA, GPIO_Pin_15);
    } else {
        GPIO_SetBits(GPIOA, GPIO_Pin_15);
    }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
    uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

    Device_Serial0 = *(uint32_t*)ID1;
    Device_Serial1 = *(uint32_t*)ID2;
    Device_Serial2 = *(uint32_t*)ID3;

    Device_Serial0 += Device_Serial2;

    if(Device_Serial0 != 0) {
        IntToUnicode(Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
        IntToUnicode(Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
    }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode(uint32_t value , uint8_t *pbuf , uint8_t len)
{
    uint8_t idx = 0;

    for(idx = 0 ; idx < len ; idx ++) {
        if(((value >> 28)) < 0xA) {
            pbuf[ 2 * idx] = (value >> 28) + '0';
        } else {
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        }

        value = value << 4;

        pbuf[ 2 * idx + 1] = 0;
    }
}

/*******************************************************************************
* Function Name  : Send DATA .
* Description    : send the data received from the STM32 to the PC through USB
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint32_t CDC_Send_DATA(uint8_t *ptrBuffer, uint8_t Send_length)
{
    /*if max buffer is Not reached*/
    if(Send_length < VIRTUAL_COM_PORT_DATA_SIZE) {
        /*Sent flag*/
        packet_sent = 0;
        /* send  packet to PMA*/
        UserToPMABufferCopy((unsigned char*)ptrBuffer, ENDP1_TXADDR, Send_length);
        SetEPTxCount(ENDP1, Send_length);
        SetEPTxValid(ENDP1);
    } else {
        return 0;
    }
    return 1;
}

#if defined(MOUNT_ENABLE) && defined(FOCUSER_ENABLE)
uint32_t ep4_send(uint8_t *ptrBuffer, uint8_t Send_length)
{
    /*if max buffer is Not reached*/
    if(Send_length < VIRTUAL_COM_PORT_DATA_SIZE) {
        /*Sent flag*/
        packet_sent = 0;
        /* send  packet to PMA*/
        UserToPMABufferCopy((unsigned char*)ptrBuffer, ENDP4_TXADDR, Send_length);
        SetEPTxCount(ENDP4, Send_length);
        SetEPTxValid(ENDP4);
    } else {
        return 0;
    }
    return 1;
}
#endif

#ifdef DEBUG
/*******************************************************************************
* Function Name  :  USART_Config_Default.
* Description    :  configure the EVAL_COM1 with default values.
* Input          :  None.
* Return         :  None.
*******************************************************************************/
void USART_Config_Default(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* Configure USART Tx as alternate function push-pull */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Configure USART Rx as input floating */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* EVAL_COM1 default configuration */
    /* EVAL_COM1 configured as follow:
          - BaudRate = 9600 baud
          - Word Length = 8 Bits
          - One Stop Bit
          - Parity Odd
          - Hardware flow control disabled
          - Receive and transmit enabled
    */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    /* USART configuration */
    USART_Init(USART1, &USART_InitStructure);

    /* Enable USART */
    USART_Cmd(USART1, ENABLE);

    /* Enable the USART Receive interrupt */
    //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}
#endif

#ifdef MOUNT_ENABLE
void tim_init(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0x8FFF;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0x8FFF;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_CtrlPWMOutputs(TIM2, ENABLE);

    /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void ra_motor_enable(uint8_t enable)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_2, (BitAction)(!enable));
}

void dec_motor_enable(uint8_t enable)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_5, (BitAction)(!enable));
}

void ra_motor_dir(uint8_t dir)
{
    GPIO_WriteBit(GPIOB, GPIO_Pin_10, (BitAction)(axis_data[RA].revert ? dir : !dir));
}

void dec_motor_dir(uint8_t dir)
{
    GPIO_WriteBit(GPIOA, GPIO_Pin_1, (BitAction)(axis_data[DEC].revert ? dir : !dir));
}

void ra_motor_mode(uint8_t mode)
{
    switch(mode) {
    case 1 :
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
        break;
    case 2 :
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
        break;
    case 4 :
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
        break;
    case 8 :
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
        break;
    case 16 :
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
        break;
    case 32 :
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_SET);
        break;
    default :
        GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
        GPIO_WriteBit(GPIOA, GPIO_Pin_4, Bit_RESET);
        break;
    }
}

void dec_motor_mode(uint8_t mode)
{
    switch(mode) {
    case 1 :
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
        break;
    case 2 :
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);
        break;
    case 4 :
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
        break;
    case 8 :
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);
        break;
    case 16 :
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
        break;
    case 32 :
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);
        break;
    default :
        GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_3, Bit_RESET);
        GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);
        break;
    }
}
#endif
