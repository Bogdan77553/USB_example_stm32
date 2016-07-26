/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
EXTI_InitTypeDef EXTI_InitStructure;

/* Extern variables ----------------------------------------------------------*/
extern __IO uint8_t PrevXferComplete;

/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len);
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power.
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
  /* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

#if defined(USB_USE_EXTERNAL_PULLUP)
  /* Enable the USB disconnect GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);
GPIO_InitTypeDef GPIO_InitStructure;
 // Пин конфигурируется как Out Push-Pull
  GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);

  // Начальное значение - лог. 1. Соответствует дисконнекту USB.
  GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);

#endif /* USB_USE_EXTERNAL_PULLUP */

  RCC_APB2PeriphClockCmd(RCC_GPIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);



 //////diodiki///////////////////
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_6;
         GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
         GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
         GPIO_Init(GPIOD, &GPIO_InitStructure);

         GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
                  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
                  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                  GPIO_Init(GPIOB, &GPIO_InitStructure);
///////////////////////////////////
// DOUT Push-pull



            GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOE, &GPIO_InitStructure);
GPIO_ResetBits(GPIOE, GPIO_Pin_2);
GPIO_ResetBits(GPIOE, GPIO_Pin_3);
GPIO_ResetBits(GPIOE, GPIO_Pin_4);
GPIO_ResetBits(GPIOE, GPIO_Pin_5);
GPIO_ResetBits(GPIOE, GPIO_Pin_6);
GPIO_ResetBits(GPIOE, GPIO_Pin_7);
GPIO_ResetBits(GPIOE, GPIO_Pin_8);
GPIO_ResetBits(GPIOE, GPIO_Pin_9);
GPIO_ResetBits(GPIOE, GPIO_Pin_10);




  /* Configure the EXTI line 18 connected internally to the USB IP ************/
  EXTI_ClearITPendingBit(EXTI_Line18);
  EXTI_InitStructure.EXTI_Line = EXTI_Line18;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}
void Delay(volatile uint32_t nCount) {
	for (; nCount != 0; nCount--);
}
void Configuration1(uint16_t *strob,uint16_t r){
GPIO_InitTypeDef  GPIO_InitStructure;
            GPIO_InitStructure.GPIO_Pin = strob[r];
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOC, &GPIO_InitStructure);

            }
void Configuration2(uint16_t *strob,uint16_t r){
    GPIO_InitTypeDef  GPIO_InitStructure;
    // а AIN(PC0) настраиваем как аналоговый вход Analog
                 GPIO_InitStructure.GPIO_Pin = strob[r];
                 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
                 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                 GPIO_Init(GPIOC, &GPIO_InitStructure);
}
uint32_t coun=0;
uint32_t coun1=0;
  uint16_t strob[16] ={
0x0001, //  GPIOE 0
0x0002,
0x0004,
0x0008,
0x0010,
0x0020,
0x0040,
0x0080,
0x0100,
0x0200,
0x0400,
0x0800,
0x1000,
0x2000,
0x4000,
0x8000,  //GPIOE 15
};

uint16_t r;
uint16_t c;

uint32_t maximal_nenazat_knopk[6][15] ={ {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
                                         {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
                                         {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
                                         {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0},
                                         {0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0}  };


void Kalibrovka(void){

 for(r=0;r<6;r++){
        if(r==4){continue;}
                  for(c=2;c<11;c++){

                      //rozrazaem
                Configuration1(strob,r);
                    GPIO_ResetBits(GPIOC, strob[r]);
                    coun=0;
                    //zarazaem
                Configuration2(strob,r);

                        GPIO_SetBits(GPIOE, strob[c]);
                    while(!GPIO_ReadInputDataBit(GPIOC, strob[r])){ coun++;}//цикл (поки(PCх)не "1")
                        GPIO_ResetBits(GPIOE, strob[c]);
                    if(coun>maximal_nenazat_knopk[r][c-2]){
                        maximal_nenazat_knopk[r][c-2]=coun+2;
                        }
                }
    }
}
uint8_t sumvol[5][15] ={   {0x28,0x29,0x2C,33,34, 0x1E,0x50,0x4F,33,34, 0,0,0,0,0},
                            {20,26,8,21,23, 20,26,8,21,23, 0,0,0,0,0},
                            {0x04,0x16,0x07,0x09,0x0A, 0x04,0x16,0x07,0x09,0x0A, 0,0,0,0,0},

                            {0x1D,0x1B,0x06,0x19,0x05, 0x1D,0x1B,0x06,0x19,0x05, 0,0,0,0,0},

                            {0x00,0x00,0x00,0x2C,0x00, 0x00,0x00,0x00,0x2C,0x00, 0,0,0,0,0}  };



/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
/* Select USBCLK source 1Div5*/
  RCC->CFGR &= (~(1<<22));
/* Select USBCLK source 1Div*/
//RCC->CFGR|= (1<<22);

  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode.
* Description    : Power-off system clocks and power while entering suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
  /* Set the device state to suspend */
  bDeviceState = SUSPENDED;

  /* Clear EXTI Line18 pending bit */
//  EXTI_ClearITPendingBit(KEY_BUTTON_EXTI_LINE);

  /* Request to enter STOP mode with regulator in low power mode */
  PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode.
* Description    : Restores system clocks and power while exiting suspend mode.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
  DEVICE_INFO *pInfo = &Device_Info;

  /* Set the device state to the correct state */
  if (pInfo->Current_Configuration != 0)
  {
    /* Device configured */
    bDeviceState = CONFIGURED;
  }
  else
  {
    bDeviceState = ATTACHED;
  }

  /*Enable SystemCoreClock*/
  SystemInit();
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config.
* Description    : Configures the USB interrupts.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* 2 bit for pre-emption priority, 2 bits for subpriority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

#if defined(STM32L1XX_MD)|| defined(STM32L1XX_HD) || defined(STM32L1XX_MD_PLUS)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#elif defined(STM32F37X)
  /* Enable the USB interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Enable the USB Wake-up interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USBWakeUp_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#else /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */
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
#endif
}


/*******************************************************************************
* Function Name  : USB_Cable_Config.
* Description    : Software Connection/Disconnection of USB Cable.
* Input          : NewState: new state.
* Output         : None.
* Return         : None
*******************************************************************************/
void USB_Cable_Config (FunctionalState NewState)
{
#if defined(STM32L1XX_MD) || defined(STM32L1XX_HD)|| defined(STM32L1XX_MD_PLUS)
  if (NewState != DISABLE)
  {
    STM32L15_USB_CONNECT;
  }
  else
  {
    STM32L15_USB_DISCONNECT;
  }

#else
  if (NewState != DISABLE)
  {
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
  else
  {
    GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
  }
#endif /* STM32L1XX_MD */
}
/*******************************************************************************
* Function Name : RHIDCheckState.
* Description   : Decodes the RHID state.
* Input         : None.
* Output        : None.
* Return value  : The state value.
*******************************************************************************/
uint16_t flg=2;
uint8_t Buffer[8]={0,0,0,0,0,0,0,0};
uint8_t RHIDCheckState(void){
    Buffer[0] = 0;
    Buffer[1] = 0;


uint32_t ii=0;

for(r=0;r<6;r++){
    if(r==4){continue;}
    for(c=2;c<11;c++){
            while(ii<50){

                           //rozrazaem
                Configuration1(strob,r);
                           GPIO_ResetBits(GPIOC, strob[r]);
                           coun=0;
                     //zarazaem
                Configuration2(strob,r);

                            GPIO_SetBits(GPIOE, strob[c]);
                        while(!GPIO_ReadInputDataBit(GPIOC, strob[r])){ coun++;}//цикл (поки(PCх)не "1")
                            GPIO_ResetBits(GPIOE, strob[c]);
                             if(coun1<coun){coun1=coun;}
                             ii++;
            }

            if(coun1<=maximal_nenazat_knopk[r][c-2]){ GPIO_ResetBits(GPIOD, GPIO_Pin_3);Buffer[flg] =0;}

                        if(coun1>maximal_nenazat_knopk[r][c-2]){
                                   GPIO_SetBits(GPIOD, GPIO_Pin_3);

                                   Buffer[flg] = sumvol[r][c-2];
                                    flg++;

                                    }
                coun1=0;
                ii=0;

                    }

}

flg=2;
    /* Reset the control token to inform upper layer that a transfer is ongoing*/
    PrevXferComplete = 0;

    /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
    USB_SIL_Write(EP1_IN, Buffer, 8);
    /* Enable endpoint for transmission*/
    SetEPTxValid(ENDP1);
    Buffer[0] = 0;
    Buffer[1] = 0;
    Buffer[2] = 0;
    Buffer[3] = 0;
    Buffer[4] = 0;
    Buffer[5] = 0;
    Buffer[6] = 0;
    Buffer[7] = 0;

    return (0);
}

/*******************************************************************************
* Function Name : RHID_Send.
* Description   : prepares buffer to be sent containing event infos.
* Input         : Keys: keys received from terminal.
* Output        : None.
* Return value  : None.
*******************************************************************************/
void RHID_Send(uint8_t report, uint8_t state)
{
  uint8_t Buffer[2] = {0, 0};

  Buffer[0] = report;
  Buffer[1] = state;

  /* Reset the control token to inform upper layer that a transfer is ongoing */
  PrevXferComplete = 0;

  /* Copy buttons data in ENDP1 Tx Packet Memory Area*/
  USB_SIL_Write(EP1_IN, Buffer, 2);
  /* Enable endpoint for transmission */
  SetEPTxValid(ENDP1);
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

  if (Device_Serial0 != 0)
  {
    IntToUnicode (Device_Serial0, &RHID_StringSerial[2] , 8);
    IntToUnicode (Device_Serial1, &RHID_StringSerial[18], 4);
  }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode (uint32_t value , uint8_t *pbuf , uint8_t len)
{
  uint8_t idx = 0;

  for( idx = 0 ; idx < len ; idx ++)
  {
    if( ((value >> 28)) < 0xA )
    {
      pbuf[ 2* idx] = (value >> 28) + '0';
    }
    else
    {
      pbuf[2* idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[ 2* idx + 1] = 0;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
