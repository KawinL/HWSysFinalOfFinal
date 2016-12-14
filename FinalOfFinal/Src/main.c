/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#define CS GPIOE
#define NCS GPIO_PIN_3
#define MSAM 1
#define Left 1
#define Right 2
#define Up 3
#define Down 4
#define X_SIZE 24
#define Y_SIZE 45
#define Bar 1
#define BarMoveLeft 1
#define BarMoveRight 2
#define BarStay 0
#define BarL 15
#define BarX 21
#define Ball 2
#define BallSX 20
#define BallSY 5
#define BallMove 1
#define BallStay 2
#define MAX_Bullet 1
#define NO_OBJ 0
#define MAX_Moveable_obj 10
#define BulletMove 1
#define Bullet 3
#define WIN 1
#define LOSE 2
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void send_uart(char* data);
void new_line();
void tab();
uint8_t get_x();
uint8_t get_y();
uint8_t get_z();
uint8_t isLoud();
uint8_t cal_sum(uint16_t data);
void Audio_Init();
void Play_note(int r);
int write_Audio_data(uint8_t address,uint8_t data);
uint8_t read_Audio_data_at(uint8_t address);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint16_t temp[1];

uint8_t note[] = {
	0x0A, 0x1A, 0x2A, 0x3A, 0x4A, 0x5A, 0x6A, 0x7A, 0x8A, 0x9A, 0xAA, 0xBA, 0xCA, 0xDA, 0xEA, 0xFA

};
uint32_t maxV=0;

typedef struct{
	uint8_t x,y,type,move;
}moveable_obj;
uint8_t map[X_SIZE][Y_SIZE+2];
moveable_obj entityList[MAX_Moveable_obj];
uint8_t n_obj =0;
uint8_t ball_dx = 1;
uint8_t ball_dy = 1;
uint8_t block[]="[XXXXXX] ";
uint8_t blockL = 9;
uint8_t n_bullet = 0;
uint8_t n_block =0;
void go(uint8_t direction){
	uint8_t data[5] ;
	data[0] = 27;
	data[1] = 91;
	data[2] = 'A';
	switch(direction){
	case Left:
		data[2]='D';
		break;
	case Right :
		data[2] = 'C';
		break;
	case Up:
		data[2] = 'A';
		break;
	case Down:
		data[2] = 'B';
		break;
	}
	HAL_UART_Transmit(&huart2,data , 3, 1000);
}



void initMap(){
	int i=0,j=0;
	for(i=0;i<X_SIZE;i++){
		for(j=0;j<Y_SIZE;j++){
			map[i][j]=' ';
		}
		map[i][j]='\n';
		map[i][j+1]='\r';
	}
	i = BarX;
	for(j=0;j<BarL;j++){
		map[i][j]='=';
	}

	for(i=4;i<8;i++){

		for(j=blockL;j<Y_SIZE  ;j++){
			map[i][j-(blockL/2)]=block[j%blockL];
			if(map[i][j-(blockL/2)]=='[')n_block++;
		}
	}


	for(i=0;i<MAX_Moveable_obj;i++){
		entityList[i].type = NO_OBJ;
	}
	n_obj=1;

	entityList[0].move=BarStay;
	entityList[0].x=BarX;
	entityList[0].y=0;
	entityList[0].type = Bar;

	map[BallSX][BallSY]='O';
	n_obj++;
	entityList[1].move=BallMove;
	entityList[1].type = Ball;
	entityList[1].x = BallSX;
	entityList[1].y= BallSY;
}
void db(uint8_t x ,uint8_t y)
{
	if(map[x][y]!=' '){
		map[x][y]=' ';
		db(x,y+1);
		db(x,y-1);
	}
}
void destroy_block(uint8_t x ,uint8_t y){
	Play_note(0);
	db(x,y);
	n_block--;
}



void updateMap(){
	int i;
	for(i=0;i<MAX_Moveable_obj;i++){
		uint8_t type = entityList[i].type;
		uint8_t* move = &entityList[i].move;
		uint8_t* x = &entityList[i].x;
		uint8_t* y = &entityList[i].y;
		switch(type){
		case Bar:
			if(*move == BarMoveLeft){
				if((*y)>0)
				{
					map[*x][(*y)+BarL-1]=' ';
					map[*x][(*y)+BarL-2]=' ';
					(*y)--;
					map[*x][*y]='=';
					(*y)--;
					map[*x][*y]='=';
				}
				*move = BarStay;
			}
			else if(*move == BarMoveRight){
				if((*y)+BarL<Y_SIZE)
				{

					map[*x][(*y)]=' ';
					(*y)++;
					map[*x][(*y)]=' ';
					(*y)++;
					map[*x][(*y)+BarL-1]='=';
					map[*x][(*y)+BarL-2]='=';
				}
				*move = BarStay;
			}
			break;
		case Ball:
			if(*move == BallMove){
				uint8_t nx,ny,insize;
				nx = (*x)+ball_dx;
				ny = (*y)+ball_dy;
				insize = 1;
				if(nx<0||nx>=X_SIZE){
					ball_dx*=-1;
					insize = 0;
					Play_note(4);
				}
				if(ny<0||ny>=Y_SIZE){
					ball_dy*=-1;
					insize = 0;
					Play_note(4);
				}
				if(insize){
					if(map[nx][ny]=='='){
						ball_dx*=-1;
						insize=0;
						Play_note(1);
					}
					else if(map[nx][ny]=='X'||map[nx][ny]=='['||map[nx][ny]==']'){
						destroy_block(nx,ny);
						ball_dx*=-1;
						insize = 0;
					}
				}
				if(insize){
					map[*x][*y]=' ';
					map[nx][ny]='O';
					*x=nx;
					*y=ny;
				}
			}
			break;
		case Bullet:
			map[*x][*y]=' ' ;
			if((*x)-1>=0){
				if(map[(*x)-1][*y]=='X'||map[(*x)-1][*y]=='['||map[(*x)-1][*y]==']'){
					destroy_block((*x)-1,*y);
					entityList[i].type = NO_OBJ;
					n_bullet--;
				}
				else {
					(*x)--;
					map[(*x)][*y]='I';
				}
			}
			else {
				n_bullet--;
				entityList[i].type = NO_OBJ;
			}
		}
	}
}

void at_bullet(){
	Play_note(8);
	uint8_t x = entityList[0].x-1;
	uint8_t y = entityList[0].y+(BarL/2)+1;
	//map[x][y]='I';
	int i=0;

	entityList[2].type = Bullet;
	entityList[2].move = BulletMove;
	entityList[2].x = x-1;
	entityList[2].y = y;


}

void updateDisplay(){
	uint8_t i;
	for(i=0;i<X_SIZE;i++){
		go(Up);
	}
	for(i=0;i<Y_SIZE;i++){
		go(Left);
	}
	for(i=0;i<X_SIZE;i++){
		HAL_UART_Transmit(&huart2,map[i],Y_SIZE,10000);
		if(i!=X_SIZE-1)HAL_UART_Transmit(&huart2,"\n\r",2,10000);
	}
}



void clearDisplay(){
	int i,j;
	for(i=0;i<X_SIZE;i++){
		for(j=0;j<Y_SIZE;j++)
		{
			map[i][j]=' ';
		}
	}
}


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
  MX_I2C1_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  Audio_Init();
  ac_Init();
  initMap();

  n_bullet = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t data;
  uint8_t x,y,z;
  uint8_t sam = MSAM;
  uint16_t pData[sam];
  int s = 0;
  int ns = 0;
  uint8_t status = 0;
  uint16_t fire = 100;
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  x= get_x();
	  if(x<235&&x>200)
	  {
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		  entityList[0].move = BarMoveLeft;
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
	  }

	  if((x>20&&x<60))
	  {
		  entityList[0].move = BarMoveRight;
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_SET);
	  }
	  else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,GPIO_PIN_RESET);


	  if(isLoud())
	  {
		  if(n_bullet==0)
		  {
			 at_bullet();
			 n_bullet++;
		  }
		  HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,1);
	  }
	  else HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,0);



	  updateMap();
	  updateDisplay();


  }


  /* USER CODE END 3 */

}

uint8_t isLoud()
{
	uint16_t pData[16];
	uint8_t sam = 8;
	HAL_I2S_Receive(&hi2s2, &pData, sam, 1000);
    int j,i,np;
    uint32_t nn=0;
    for(j=0;j<sam;j++)
    {
	  uint16_t temp =pData[j];
	  nn+= cal_sum(temp);
    }
    HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_15);

    return nn>43;
}

void make_wave(uint8_t n,char* t)
{

	int i = 0;
	for(i=0;i<16;i++)
	{
		t[i]=' ';
	}
	t[i]='\0';
	if(n>=8)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,1);
		return;
	}

	if(n<4)
	{
		for(i=n;i<=3;i++)
		{
			t[i]='|';
		}
	}
	else
	{
		for(i = 4;i<=n;i++)
		{
			t[i]='|';
		}
	}

}
uint8_t cal_sum(uint16_t data)
{
	int i;
	uint8_t n=0;
	for(i=0;i<16;i++)
	{
		n+=data%2;
		data/=2;
	}
	return n;
}
void ac_Init()
{
      HAL_GPIO_WritePin(CS,NCS,0);
	  uint8_t data[]={ 0x20,0x47};
	  HAL_SPI_Transmit(&hspi1,&data,2,1000);
	  HAL_GPIO_WritePin(CS,NCS,1);
}
uint8_t get_x()
{
	  uint8_t d;
	  uint8_t data[2];
	  HAL_GPIO_WritePin(CS,NCS,0);
	  data[0] = 0b10101001;
	  HAL_SPI_Transmit(&hspi1,&data[0],1,100);
	  HAL_SPI_Receive(&hspi1,&d,1,100);
	  HAL_GPIO_WritePin(CS,NCS,1);
	  return d;
}
uint8_t get_y()
{
	  uint8_t d;
	  uint8_t data[2];
	  HAL_GPIO_WritePin(CS,NCS,0);
	  data[0] = 0x2B+ 0x80;
	  HAL_SPI_Transmit(&hspi1,&data[0],1,100);
	  HAL_SPI_Receive(&hspi1,&d,1,100);
	  HAL_GPIO_WritePin(CS,NCS,1);
	  return d;
}
uint8_t get_z()
{
	  uint8_t d;
	  uint8_t data[2];
	  HAL_GPIO_WritePin(CS,NCS,0);
	  data[0] = 0x2D + 0x80;
	  HAL_SPI_Transmit(&hspi1,&data[0],1,100);
	  HAL_SPI_Receive(&hspi1,&d,1,100);
	  HAL_GPIO_WritePin(CS,NCS,1);
	  return d;
}
int write_Audio_data(uint8_t address,uint8_t data){
	uint8_t cawrite = 0b10010100;
	uint8_t send_data[2];
	send_data[0]=address;
	send_data[1]=data;
	return HAL_I2C_Master_Transmit(&hi2c1,cawrite,send_data,2,50);
}

uint8_t read_Audio_data_at(uint8_t address){
	uint8_t caread = 0b10010101;
	uint8_t cawrite = 0b10010100;
	HAL_I2C_Master_Transmit(&hi2c1,cawrite,&address,1,10000);
	HAL_I2C_Master_Receive(&hi2c1,caread,&address,1,1000);
	return address;
}

void Play_note(int r){

	if(r < 0 || r > 9) return;

	write_Audio_data(0x1E,0x20);

	write_Audio_data(0x1C,note[r]);

	write_Audio_data(0x1E,0xE0);

	int i;

	for(i=0;i<100;i++)HAL_I2S_Transmit (&hi2s3, temp , 100, 10 );
}

void Audio_Init(){

	  temp[0] = 0;

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 0);

	  HAL_Delay(500);

	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1); //Reset is set Up (Power CS43L22)
	  HAL_Delay(500);//GPIOD  GPIO_PIN_4

	  write_Audio_data(0x00,0x99);

	  write_Audio_data(0x47,0x80);

	  write_Audio_data(0x32,0x80);

	  write_Audio_data(0x32,0x00);

	  write_Audio_data(0x00,0x00);

	  write_Audio_data(0x1E,0xC0);

	  write_Audio_data(0x02,0x9E);

	  //write_Audio_data(0x1D,0x07);

}
void send_uart(char* data)
{
	int i=0;
	while(data[i]!='\0')
	{
		HAL_UART_Transmit(&huart2,&data[i],1,100);
		i++;
	}
}

void new_line()
{
	HAL_UART_Transmit(&huart2,"\n\r",2,100);
}

void tab()
{
	HAL_UART_Transmit(&huart2,"\t",2,100);
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 88;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S2 init function */
static void MX_I2S2_Init(void)
{

  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_192K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT1_Pin */
  GPIO_InitStruct.Pin = MEMS_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

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
