/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 XYZ
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_tx;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
  unsigned short timer=0;//定时累计值

	unsigned char led_num=0;//LED列号

unsigned char pinflag=0xff;//339比较值标记  0:低比较值，  ff:高比较值
	
	unsigned char keyMem[11][8]={0,0,0,0,0,0,0,0,0,0,0};//=0; 88个按键状态存储
	unsigned short  keyTimer[11][8];//88按键的每个第一按键的时间

	unsigned char   uartbuf_key[200]={0};//串行发送值缓冲区
	unsigned char   bufnum_key=0;//串行缓冲存入字节序号
	
	unsigned char   uart_dma_buf_long_key=0;//串行缓冲没有发送的字节数
	unsigned char   dma_flag_key=0;//单次DMA发送完成标记
	unsigned char   uart_dma_buf_out_key=0;//串行缓冲经DMA发送字节序号
	
	unsigned char   uartbuf_midi[200]={0};//串行发送值缓冲区
	unsigned char   bufnum_midi=0;//串行缓冲存入字节序号
	
	unsigned char   uart_dma_buf_long_midi=0;//串行缓冲没有发送的字节数
	unsigned char   dma_flag_midi=0;//单次DMA发送完成标记
	unsigned char   uart_dma_buf_out_midi=0;//串行缓冲经DMA发送字节序号
	
	
	unsigned char   chk_flg=0;	  //定时扫描按键标记
	unsigned short  key_send_flg[8]={0};//按键确立标记	
	
	
	
	
	uint16_t                      aResultDA[6];
	uint8_t  ad_flag=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void channel_sel(unsigned char channel_on);//按键通道扫描
static void MX_ADC_Init(void);
//static uint16_t get_ave(uint16_t *bufpoint);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
	MX_ADC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
	MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET)	;//关断三极管, 输出高的比较电压  
	pinflag  = 0;	
	channel_sel(led_num);
	
//	HAL_ADC_Start_DMA(&hadc, (uint32_t *)&aResultDA[0], 6);					
  while (1)
  {
  /* USER CODE END WHILE */
   if(chk_flg)
		{				
			chk_flg=0;
			if(pinflag)//第1键位比较
			{
				 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10))//变高,有输出 芯片1 
				 {
					 if((keyMem[0][led_num]&0x01)==0)
					 {
						  keyMem[0][led_num] |=0x01;
						  keyTimer[0][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[0][led_num]&0x01)&&(key_send_flg[led_num]&0x01))
					 {							 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=led_num;
				      uartbuf_key[bufnum_key++]=0;	
              key_send_flg[led_num] &=~0x01;	  
						  if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;
						 
						  uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;						 
					 }
            keyMem[0][led_num] =0;				 
				 }
	////////////////////////////			 
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))//变高,有输出 芯片2
				 {
					 if((keyMem[1][led_num]&0x01)==0)
					 {
						  keyMem[1][led_num] |=0x01;
						  keyTimer[1][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[1][led_num]&0x01)&&(key_send_flg[led_num]&0x02))
					 {	
             // uartbuf_key[bufnum_key++]=0x09;						 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=8+led_num;//+21;;
				      uartbuf_key[bufnum_key++]=0;	
              key_send_flg[led_num] &=~0x02;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;												 
					 }	
           keyMem[1][led_num] =0;					 
				 }
//////////////////////////////////////////		
       if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))//变高,有输出 芯片3
				 {
					 if((keyMem[2][led_num]&0x01)==0)
					 {
						  keyMem[2][led_num] |=0x01;
						  keyTimer[2][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[2][led_num]&0x01)&&(key_send_flg[led_num]&0x04))
					 {
              //uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=16+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;
              key_send_flg[led_num] &=~0x04;			
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;												 
					 }	
           keyMem[2][led_num] =0;					 
				 }
/////////////////////////////////////			
        if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))//变高,有输出 芯片4//////////????????
				 {
					 if((keyMem[3][led_num]&0x01)==0)
					 {
						  keyMem[3][led_num] |=0x01;
						  keyTimer[3][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[3][led_num]&0x01)&&(key_send_flg[led_num]&0x08))
					 {	
              //uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=24+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;		
              key_send_flg[led_num] &=~0x08;	
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4; 

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
					 }	
           keyMem[3][led_num] =0;					 
				 }				 
///////////////////////////////////////////
			 if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))//变高,有输出 芯片5
         {
					 if((keyMem[4][led_num]&0x01)==0)
					 {
						  keyMem[4][led_num] |=0x01;
						  keyTimer[4][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[4][led_num]&0x01)&&(key_send_flg[led_num]&0x10))
					 {				
              //uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=32+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;		
              key_send_flg[led_num] &=~0x10;
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
					 }	
           keyMem[4][led_num] =0;					 
				 }				 
///////////////////////////////////////////
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))//变高,有输出 芯片6		 
        {
					 if((keyMem[5][led_num]&0x01)==0)
					 {
						  keyMem[5][led_num] |=0x01;
						  keyTimer[5][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[5][led_num]&0x01)&&(key_send_flg[led_num]&0x20))
					 {						 
              //uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=40+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;		
              key_send_flg[led_num] &=~0x20;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
					 }	
           keyMem[5][led_num] =0;					 
				 }			
//////////////////////////////////////////////////////////
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))//变高,有输出 芯片7	
				 {
					 if((keyMem[6][led_num]&0x01)==0)
					 {
						  keyMem[6][led_num] |=0x01;
						  keyTimer[6][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[6][led_num]&0x01)&&(key_send_flg[led_num]&0x40))
					 {			
             // uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=48+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;		
              key_send_flg[led_num] &=~0x40;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
					 }	
           keyMem[6][led_num] =0;					 
				 }				 
///////////////////////////////////////////
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))//变高,有输出 芯片8
				{
					 if((keyMem[7][led_num]&0x01)==0)
					 {
						  keyMem[7][led_num] |=0x01;
						  keyTimer[7][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[7][led_num]&0x01)&&(key_send_flg[led_num]&0x80))
					 {					
              //uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=56+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;		
              key_send_flg[led_num] &=~0x80;	
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
					 }	
           keyMem[7][led_num] =0;					 
				 }				 
///////////////////////////////////////////
			if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7))//变高,有输出 芯片9
        {
					 if((keyMem[8][led_num]&0x01)==0)
					 {
						  keyMem[8][led_num] |=0x01;
						  keyTimer[8][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[8][led_num]&0x01)&&(key_send_flg[led_num]&0x100))
					 {						 
              //uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=64+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;		
              key_send_flg[led_num] &=~0x100;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
					 }	
           keyMem[8][led_num] =0;					 
				 }				 
///////////////////////////////////////////
			if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6))//变高,有输出 芯片10
        {
					 if((keyMem[9][led_num]&0x01)==0)
					 {
						  keyMem[9][led_num] |=0x01;
						  keyTimer[9][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[9][led_num]&0x01)&&(key_send_flg[led_num]&0x200))
					 {			
              //uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=72+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;		
              key_send_flg[led_num] &=~0x200;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
					 }	
           keyMem[9][led_num] =0;					 
				 }
///////////////////////////////////////////
			if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12))//变高,有输出 芯片11
				{
					 if((keyMem[10][led_num]&0x01)==0)
					 {
						  keyMem[10][led_num] |=0x01;
						  keyTimer[10][led_num]=timer;						 
					 }					 
				 }
				 else
				 {
					 if((keyMem[10][led_num]&0x01)&&(key_send_flg[led_num]&0x400))
					 {					
             // uartbuf_key[bufnum_key++]=0x09;									 
						  uartbuf_key[bufnum_key++]=0x90;
				      uartbuf_key[bufnum_key++]=80+led_num;//+21;
				      uartbuf_key[bufnum_key++]=0;		
              key_send_flg[led_num] &=~0x400;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=0;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
					 }	
           keyMem[10][led_num] =0;					 
				 }

///////////////////////////////////				 
			}
		else   //第2键位比较
			{
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10))//变高,有输出 芯片1
				 {
					 if(keyMem[0][led_num]==0x01)
					 {
						  keyMem[0][led_num] |=0x10;
						  if(keyTimer[0][led_num]<timer)  keyTimer[0][led_num] = timer-keyTimer[0][led_num];
			      	else                  keyTimer[0][led_num] = timer+(65535-keyTimer[0][led_num]);
				      //uartbuf_key[bufnum_key++]=0x09;			
						  uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[0][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x01;	
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;
						 
              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[0][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;														 
                  						 
					 }					 
				 }
		////////////////		 /////////////////
				if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11))//变高,有输出 芯片2
				 {
					 if(keyMem[1][led_num]==0x01)
					 {
						  keyMem[1][led_num] |=0x10;
						  if(keyTimer[1][led_num]<timer)  keyTimer[1][led_num] = timer-keyTimer[1][led_num];
			      	else                  keyTimer[1][led_num] = timer+(65535-keyTimer[1][led_num]);
             // uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=8+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[1][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x02;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=8+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[1][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
				 }
	//////////////////////////////////////////////////////
       if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7))//变高,有输出 芯片3
				 {
					 if(keyMem[2][led_num]==0x01)
					 {
						  keyMem[2][led_num] |=0x10;
						  if(keyTimer[2][led_num]<timer)  keyTimer[2][led_num] = timer-keyTimer[2][led_num];
			      	else                  keyTimer[2][led_num] = timer+(65535-keyTimer[2][led_num]);
              //uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=16+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[2][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x04;					
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;		

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=16+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[2][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
				 }
///////////////////////////////////////
       if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6))//变高,有输出 芯片4
				 {
					  if(keyMem[3][led_num]==0x01)
					 {
						  keyMem[3][led_num] |=0x10;
						  if(keyTimer[3][led_num]<timer)  keyTimer[3][led_num] = timer-keyTimer[3][led_num];
			      	else                  keyTimer[3][led_num] = timer+(65535-keyTimer[3][led_num]);
              //uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=24+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[3][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x08;			
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;		

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=24+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[3][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
				 }
///////////////////////////////////////////
			if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))//变高,有输出 芯片5
      	 {
					  if(keyMem[4][led_num]==0x01)
					 {
						  keyMem[4][led_num] |=0x10;
						  if(keyTimer[4][led_num]<timer)  keyTimer[4][led_num] = timer-keyTimer[4][led_num];
			      	else                  keyTimer[4][led_num] = timer+(65535-keyTimer[4][led_num]);
              //uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=32+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[4][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x10;
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=32+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[4][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
				 }
///////////////////////////////////////////
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))//变高,有输出 芯片6			
      {
					  if(keyMem[5][led_num]==0x01)
					 {
						  keyMem[5][led_num] |=0x10;
						  if(keyTimer[5][led_num]<timer)  keyTimer[5][led_num] = timer-keyTimer[5][led_num];
			      	else                  keyTimer[5][led_num] = timer+(65535-keyTimer[5][led_num]);
             // uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=40+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[5][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x20;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;			

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=40+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[5][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
				 } 	
//////////////////////////////////////////////////////////
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3))//变高,有输出 芯片7	
      {
					  if(keyMem[6][led_num]==0x01)
					 {
						  keyMem[6][led_num] |=0x10;
						  if(keyTimer[6][led_num]<timer)  keyTimer[6][led_num] = timer-keyTimer[6][led_num];
			      	else                  keyTimer[6][led_num] = timer+(65535-keyTimer[6][led_num]);
             // uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=48+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[6][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x40;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=led_num+21;
				      uartbuf_midi[bufnum_midi++]=48+127-keyTimer[6][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
			 } 	 		
///////////////////////////////////////////
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))//变高,有输出 芯片8	
       {
					  if(keyMem[7][led_num]==0x01)
					 {
						  keyMem[7][led_num] |=0x10;
						  if(keyTimer[7][led_num]<timer)  keyTimer[7][led_num] = timer-keyTimer[7][led_num];
			      	else                  keyTimer[7][led_num] = timer+(65535-keyTimer[7][led_num]);
              //uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=56+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[7][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x80;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=56+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[7][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
			 }
///////////////////////////////////////////
	 if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_7))//变高,有输出 芯片9			
		 {
					  if(keyMem[8][led_num]==0x01)
					 {
						  keyMem[8][led_num] |=0x10;
						  if(keyTimer[8][led_num]<timer)  keyTimer[8][led_num] = timer-keyTimer[8][led_num];
			      	else                  keyTimer[8][led_num] = timer+(65535-keyTimer[8][led_num]);
             // uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=64+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[8][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x100;		
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;	

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=64+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[8][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
			 }
///////////////////////////////////////////
		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6))//变高,有输出 芯片10		 
		  {
					  if(keyMem[9][led_num]==0x01)
					 {
						  keyMem[9][led_num] |=0x10;
						  if(keyTimer[9][led_num]<timer)  keyTimer[9][led_num] = timer-keyTimer[9][led_num];
			      	else                  keyTimer[9][led_num] = timer+(65535-keyTimer[9][led_num]);
             // uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=72+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[9][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x200;	
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;		

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=72+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[9][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
			  }
///////////////////////////////////////////
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12))//变高,有输出 芯片11		
   		{
					  if(keyMem[10][led_num]==0x01)
					 {
						  keyMem[10][led_num] |=0x10;
						  if(keyTimer[10][led_num]<timer)  keyTimer[10][led_num] = timer-keyTimer[10][led_num];
			      	else                  keyTimer[10][led_num] = timer+(65535-keyTimer[10][led_num]);
              //uartbuf_key[bufnum_key++]=0x09;			
				      uartbuf_key[bufnum_key++]=0x90;
			      	uartbuf_key[bufnum_key++]=80+led_num;//+21;
				      uartbuf_key[bufnum_key++]=127-keyTimer[10][led_num]/64;///64;			
             // uartbuf_key[bufnum_key++]=key1_timer%256;	
               key_send_flg[led_num] |=0x400;
              if(bufnum_key>197)  bufnum_key=0;
						  uart_dma_buf_long_key +=3;//4;		

              uartbuf_midi[bufnum_midi++]=0x09;
  						uartbuf_midi[bufnum_midi++]=0x90;
				      uartbuf_midi[bufnum_midi++]=80+led_num+21;
				      uartbuf_midi[bufnum_midi++]=127-keyTimer[10][led_num]/64;;
						  if(bufnum_midi>199)  bufnum_midi=0;
						  uart_dma_buf_long_midi += 4;									 
					 }						 
			  }
///////////////////////////////////////////////////////				 
	}//else duiying
///////////////////////	一组扫描完成
			if(++led_num>7)  
				{
					led_num=0;
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
        	pinflag  =~pinflag;	
          HAL_ADC_Start_DMA(&hadc, (uint32_t *)&aResultDA[0], 6);					
				}
			channel_sel(led_num);
	  }
  /* USER CODE BEGIN 3 */
   
///////////////////////////////////////////////////转发检查
   if((uart_dma_buf_long_midi)&&(dma_flag_midi==0))//发送MIDI检查
	  {			
			  dma_flag_midi=1;
	  	  if(HAL_UART_Transmit_DMA(&huart2, &uartbuf_midi[uart_dma_buf_out_midi], uart_dma_buf_long_midi)==HAL_OK)	
				 {					
		       uart_dma_buf_out_midi += uart_dma_buf_long_midi;
		       if(uart_dma_buf_out_midi>199) uart_dma_buf_out_midi=0;
		       uart_dma_buf_long_midi =0;
		     }
				else  dma_flag_midi=0;
	  }  	
  if((uart_dma_buf_long_key)&&(dma_flag_key==0))//发送按键检查
	  {			
			 dma_flag_key=1;
	  	 if(HAL_UART_Transmit_DMA(&huart1, &uartbuf_key[uart_dma_buf_out_key], uart_dma_buf_long_key)==HAL_OK)	
				 {					 
		       uart_dma_buf_out_key += uart_dma_buf_long_key;
		       if(uart_dma_buf_out_key>197) uart_dma_buf_out_key=0;
		       uart_dma_buf_long_key =0;
		     }
				 else  dma_flag_key=0;
	  }		
    if(ad_flag)
		{
			ad_flag=0;
//			HAL_ADC_Stop_DMA(&hadc);	
//			if(get_ave(aResultDA)>500)
//			{
//				 uartbuf_key[bufnum_key++]=0xB0;
//				 uartbuf_key[bufnum_key++]=0x40;//40,42;43
//				 uartbuf_key[bufnum_key++]=62;	//1~63//64~127
//         if(bufnum_key>197)  bufnum_key=0;
//				 uart_dma_buf_long_key +=3;//4; 
//				
//			}					
		}
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 100;//239;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);
}
/* USART2 init function */
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
  huart2.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);

}
/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
	
	HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}
//////////////////////////////////
/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __GPIOB_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
	
	GPIO_InitTypeDef GPIO_InitStructure;	
	
	GPIO_InitStructure.Pin   =  GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 ;
	GPIO_InitStructure.Speed =  GPIO_SPEED_HIGH  ;	
	GPIO_InitStructure.Mode  =  GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);	
		
	GPIO_InitStructure.Pin   =  GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8  | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15 ;
	GPIO_InitStructure.Pull =   GPIO_PULLDOWN;		
	GPIO_InitStructure.Mode  =  GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);	

   /* Configure PB0,1 pin as input floating */
 // GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
 // GPIO_InitStructure.Pull = GPIO_NOPULL;
 // GPIO_InitStructure.Pin =  GPIO_PIN_0 | GPIO_PIN_1;
 // HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

  /* Enable and set EXTI line 0_1 Interrupt to the lowest priority */
//  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 2, 0);
 // HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);	

	GPIO_InitStructure.Pin   =  GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 ;
	GPIO_InitStructure.Pull =   GPIO_PULLDOWN;	
	GPIO_InitStructure.Mode  =  GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);		
	
	GPIO_InitStructure.Pin   =  GPIO_PIN_9;
	GPIO_InitStructure.Speed =  GPIO_SPEED_HIGH  ;	
	GPIO_InitStructure.Mode  =  GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	GPIO_InitStructure.Pin   =  GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 ;
	GPIO_InitStructure.Pull =   GPIO_PULLDOWN;		
	GPIO_InitStructure.Mode  =  GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	GPIO_InitStructure.Pin   =  GPIO_PIN_6 | GPIO_PIN_7 ;
	GPIO_InitStructure.Pull =   GPIO_PULLDOWN;		
	GPIO_InitStructure.Mode  =  GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStructure);	

}

/////////////////////////////
/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION10b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode =  ENABLE;//DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = OVR_DATA_PRESERVED;
  HAL_ADC_Init(&hadc);
	
	//Start calibration
	HAL_ADCEx_Calibration_Start(&hadc);

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}


////////////////////
/* USER CODE BEGIN 4 */
void channel_sel(unsigned char channel_on)
{
	switch (channel_on)
	{
		 case 0 :			  
			   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET)	;		   
		     break;
		 case 1 :			  
			   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET)	;		   
		     break;
		 case 2 :			  
			   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET)	;		   
		     break;
		 case 3 :			  
			   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET)	;		   
		     break;
		 case 4 :			  
			   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET)	;		   
		     break;
		 case 5 :			  
			   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET)	;		   
		     break;
		 case 6 :			  
			   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET)	;		   
		     break;
		  case 7 :			  
			   HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET)	;
		     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_SET)	;		   
		     break;
		 default : break;		
	}
	
}

/* USER CODE END 4 */
///////////////////////////
uint16_t get_ave(uint16_t *bufpoint)
{
	uint16_t min_tmp=*bufpoint;
	uint16_t max_tmp=*bufpoint;
	uint16_t sum=*bufpoint;
	uint8_t i;
	for(i=1;i<6;i++)
	{
		if(*(bufpoint+i)>max_tmp)  max_tmp=*(bufpoint+i);
		if(*(bufpoint+i)<min_tmp)  min_tmp=*(bufpoint+i);		
		sum += *(bufpoint+i);
	}
	
	sum=sum-min_tmp-max_tmp;
	sum=sum/4;
	
	return sum;
}
////////////////////////////

//uint8_t find_vole(uint16_t timevole,uint16_t *pointt)
//{
//	uint8_t i;
//	if(timevole>*(pointt+63))
//	{
//		for(i=63;i<127;i++)
//		{
//			if(timevole>*(pointt+i)) return i;			
//		}
//	}
//	else 
//	{
//		for(i=0;i<63;i++)
//		{
//			if(timevole>*(pointt+i)) return i;			
//		}			
//	}	
//}
#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
