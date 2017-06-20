/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32l4xx_hal.h"
#include "lcd.h"
#include "quadspi.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stm32l476g_discovery_glass_lcd.h"
#include "stm32l476g_discovery_gyroscope.h"
#include "stm32l476g_discovery_compass.h"
#include "stm32l476g_discovery_qspi.h"
#define ABS(x)                    (x < 0) ? (-x) : x
#define ACC_THRESHOLD_DETECTION 50
#define HEX_2_DEC(val) (((val)/16)*10+((val)%16))
#define DEC_2_HEX(val) (((val)/10)*16+((val)%10))
#define max(a,b) (a>=b?a:b)
#define min(a,b) (a<=b?a:b)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
  volatile int liczbadanych;
  volatile int tablicadanych[1000];
  volatile int tablicaczasow[1000];
  volatile int cowyswietlic=6;
  volatile int pierwszeuruchomienie=1;
  volatile int flaginterrupt=1;
  volatile int displaycase=0;
uint32_t cm=0;
int16_t axisXYZ[3];
/* Gyroscope variables */
int16_t buffer[3] = {0};
 int32_t xval = 0;
 int32_t yval = 0;
 int32_t zval = 0;
 uint32_t xvalabs = 0;
 uint32_t yvalabs = 0;
 uint32_t zvalabs = 0;
 uint32_t acceleval = 0;
int32_t xs = 0;
int32_t ys = 0;
int32_t zs = 0;
int32_t xv = 0;
int32_t yv = 0;
int32_t zv = 0;
int32_t xoff = 0;
int32_t yoff = 0;
int32_t zoff = 0;
int32_t vvv = 0;
  uint16_t datatodisplay[LCD_DIGIT_MAX_NUMBER] = {0};
  uint8_t i = 0;
#define displaycase_up 2
#define displaycase_down 3
#define displaycase_left 1
#define displaycase_center 4
#define displaycase_right 5



  kompas backup;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void Gyro_Convert(uint8_t Axis, int32_t Value, uint16_t *DisplayString);
void Compass_Calib(kompas *backup);
void Compass_Run(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if((GPIO_Pin & SEL_JOY_PIN)!= RESET && flaginterrupt==0) flaginterrupt = 1;
	if((GPIO_Pin & SEL_JOY_PIN)!= RESET && flaginterrupt==1) displaycase = displaycase_center;
	if((GPIO_Pin & LEFT_JOY_PIN)!= RESET) displaycase = displaycase_left;
	if((GPIO_Pin & RIGHT_JOY_PIN)!= RESET) displaycase = displaycase_right;
	if((GPIO_Pin & UP_JOY_PIN)!= RESET) displaycase = displaycase_up;
	if((GPIO_Pin & DOWN_JOY_PIN)!= RESET) displaycase = displaycase_down;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim==&htim6)
	{
		if(flaginterrupt==0) flaginterrupt=2;
	}
}

void Convert_IntegerIntoChar(uint32_t number, uint16_t *p_tab)
{
  uint16_t units=0, tens=0, hundreds=0, thousands=0, tenthousand=0, hundredthousand=0;

  units = ((((number%100000)%10000)%1000)%100)%10;
  tens = (((((number-units)/10)%10000)%1000)%100)%10;
  hundreds = ((((number-tens-units)/100)%1000)%100)%10;
  thousands = (((number-hundreds-tens-units)/1000)%100)%10;
  tenthousand = ((number-thousands-hundreds-tens-units)/10000)%10;
  hundredthousand = ((number-tenthousand-thousands-hundreds-tens-units)/100000);

  *(p_tab+5) = units + 0x30;
  *(p_tab+4) = tens + 0x30;
  *(p_tab+3) = hundreds + 0x30;
  *(p_tab+2) = thousands + 0x30;
  *(p_tab+1) = tenthousand + 0x30;
  *(p_tab+0) = hundredthousand + 0x30;
}

int32_t sq_rt(int32_t val)
{
	int32_t root = 0;

	if(val < 0)
		return root;
	while(root*root < val)
	{
		root++;
	}
	return root;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	liczbadanych=0;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LCD_Init();
  MX_TIM6_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_QUADSPI_Init();

  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED4_GPIO_PORT, LED4_PIN, SET);
  HAL_TIM_Base_Start_IT(&htim6);
  BSP_LCD_GLASS_Init();
  if (BSP_COMPASS_Init() == COMPASS_ERROR)
  {
	  BSP_LCD_GLASS_DisplayString("err");
	  while (1){}
  }
  if (BSP_GYRO_Init() == GYRO_ERROR)
    {
  	  BSP_LCD_GLASS_DisplayString("err");
  	  while (1){}
    }
  if (BSP_QSPI_Init() != QSPI_OK)
	{
	  BSP_LCD_GLASS_DisplayString("err");
	  while (1){}
	}
  BSP_JOY_Init(JOY_MODE_EXTI);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //calibration

  for (int i=0; i<128; i++)
  {
	  BSP_COMPASS_AccGetXYZ(buffer);
	  xval = (int32_t) buffer[0];
	  yval = (int32_t) buffer[1];
	  zval = (int32_t) buffer[2];
	  xoff += xval;
	  yoff += yval;
	  zoff += zval;
	  HAL_Delay(5);
  }
  xoff = xoff >> 7;
  yoff = yoff >> 7;
  zoff = zoff >> 7;
  //Compass_Calib(&backup);
  HAL_GPIO_WritePin(LED4_GPIO_PORT, LED4_PIN, RESET);
  //Compass_Run();
  int tick;
  while (1)
  {
	  if(flaginterrupt==0)
	  {
		  tick=HAL_GetTick();
		  BSP_COMPASS_AccGetXYZ(buffer);

		  /* Get absolute value */
		  xval = (((int32_t) buffer[0])-xoff)*61/1000;
		  yval = (((int32_t) buffer[1])-yoff)*61/1000;
		  zval = (((int32_t) buffer[2])-zoff)*61/1000;
		  xvalabs = ABS(xval);
		  yvalabs = ABS(yval);
		  zvalabs = ABS(zval);
		  if(xvalabs < ACC_THRESHOLD_DETECTION) xval=0;
		  if(yvalabs < ACC_THRESHOLD_DETECTION) yval=0;
		  if(zvalabs < ACC_THRESHOLD_DETECTION) zval=0;
		  xv+=xval/100;
		  yv+=yval/100;
		  zv+=zval/100;
		  acceleval=sq_rt(xvalabs*xvalabs+yvalabs*yvalabs+zvalabs*zvalabs);
		  vvv=sq_rt(xv*xv+yv*yv+zv*zv);
		  /* reset display buffer */
		  for(i = 0; i < LCD_DIGIT_MAX_NUMBER; i++)
		  {
			datatodisplay[i] = 0;
		  }
		  if(cowyswietlic==0) Gyro_Convert('A', acceleval, datatodisplay);
		  if(cowyswietlic==1) Gyro_Convert('V', vvv, datatodisplay);
		  if(cowyswietlic==2) Gyro_Convert('X', xvalabs, datatodisplay);
		  if(cowyswietlic==3) Gyro_Convert('Y', yvalabs, datatodisplay);
		  if(cowyswietlic==4) Gyro_Convert('Z', zvalabs, datatodisplay);
		  BSP_LCD_GLASS_DisplayStrDeci(datatodisplay);
		  while((HAL_GetTick() - tick) < 2)
		  {
		  }
		  /* insert Delay */
		  HAL_Delay(2);
	  }
	  else if(flaginterrupt==1)//przycisk i wyswietlanie
	  {
		  BSP_LCD_GLASS_ScrollSentence("       PAUSE",1,SCROLL_SPEED_HIGH);
		  BSP_LCD_GLASS_Clear();
		  displaycase=0;
		  HAL_Delay(50);
		  while(displaycase!=displaycase_center && !pierwszeuruchomienie)
		  {
			  if(displaycase==displaycase_up)
			  {
				  displaycase=0;
				  cowyswietlic--;
				  if (cowyswietlic<0) cowyswietlic=6;
				  HAL_Delay(50);
			  }
			  if(displaycase==displaycase_down)
			  {
				  displaycase=0;
				  cowyswietlic++;
				  if (cowyswietlic>6) cowyswietlic=0;
				  HAL_Delay(50);
			  }
			  HAL_Delay(50);
			  if(cowyswietlic==0)
			  {
				  BSP_LCD_GLASS_Clear();
				  BSP_LCD_GLASS_DisplayString("  Acc ");
			  }
			  if(cowyswietlic==1)
			  {
				  BSP_LCD_GLASS_Clear();
				  BSP_LCD_GLASS_DisplayString("  Vel");
			  }
			  if(cowyswietlic==2)
			  {
				  BSP_LCD_GLASS_Clear();
				  BSP_LCD_GLASS_DisplayString("  Ax");
			  }
			  if(cowyswietlic==3)
			  {
				  BSP_LCD_GLASS_Clear();
				  BSP_LCD_GLASS_DisplayString("  Ay");
			  }
			  if(cowyswietlic==4)
			  {
				  BSP_LCD_GLASS_Clear();
				  BSP_LCD_GLASS_DisplayString("  Az");
			  }
			  if(cowyswietlic==5)
			  {
				  BSP_LCD_GLASS_Clear();
				  BSP_LCD_GLASS_DisplayString(" Hist");
			  }
			  if(cowyswietlic==6)
			  {
				  BSP_LCD_GLASS_Clear();
				  BSP_LCD_GLASS_DisplayString(" Time");
			  }

		  }
		  if(cowyswietlic==5)
		  {
			  int i=liczbadanych;
			  uint16_t *DisplayString;
			  int valortime=0;
			  do
			  {
				  BSP_LCD_GLASS_Clear();
				  for(int j = 0; j < LCD_DIGIT_MAX_NUMBER; j++)
				  {
					  datatodisplay[i] = 0;
				  }
				  if(valortime==0)
				  {
					  uint8_t *ptr;
					  BSP_QSPI_Read(ptr,i*N25Q128A_SUBSECTOR_SIZE,0x1000);
					  //Gyro_Convert('H', tablicadanych[i], datatodisplay);
					  Gyro_Convert('H', ptr, datatodisplay);
					  BSP_LCD_GLASS_DisplayStrDeci(datatodisplay);
				  }
				  if(valortime==1)
				  {
					  int hh=tablicaczasow[i]/10000;
					  int mm=(tablicaczasow[i]-hh*10000)/100;
					  int ss=tablicaczasow[i]-hh*10000-mm*100;
					  char text[6] = {0};
					  sprintf(text,"%.2u%.2u%.2u",hh,mm,ss);
					  BSP_LCD_GLASS_DisplayString(text);
				  }

				  if(displaycase==displaycase_up)
				  {
					  displaycase=0;
					  i--;
					  if (i<0) i=liczbadanych;
					  HAL_Delay(50);
				  }
				  if(displaycase==displaycase_down)
				  {
					  displaycase=0;
					  i++;
					  if (i>liczbadanych) i=0;
					  HAL_Delay(50);
				  }
				  if(displaycase==displaycase_right)
				  {
					  displaycase=0;
					  if(valortime==0) valortime=1;
					  else if(valortime==1) valortime=0;
					  HAL_Delay(50);
				  }
				  HAL_Delay(50);
			  } while(displaycase!=displaycase_left);
			  cowyswietlic=0;
		  }
		  if(cowyswietlic==6)
		  {
			  int hour=23;
			  int minute=59;
			  int second=59;
			  int ifh=0, ifm=0, ifs=0;
			  int tm=10000*hour+100*minute+second;
			  displaycase=0;
			  do
			  {
				  HAL_Delay(50);
				  BSP_LCD_GLASS_Clear();
				  for(int j = 0; j < LCD_DIGIT_MAX_NUMBER; j++) datatodisplay[i] = 0;
				  Gyro_Convert('TH', hour, datatodisplay);
				  BSP_LCD_GLASS_DisplayStrDeci(datatodisplay);
				  if(displaycase==displaycase_up)
				  {
					  displaycase=0;
					  hour--;
					  if(hour<0) hour=23;
					  HAL_Delay(50);
				  }
				  if(displaycase==displaycase_down)
				  {
					  displaycase=0;
					  hour++;
					  if(hour>23) hour=0;
					  HAL_Delay(50);
				  }
				  tm=10000*hour+100*minute+second;
			  }while(displaycase!=displaycase_center);
			  displaycase=0;
			  do
			  			  {
			  				  HAL_Delay(50);
			  				  BSP_LCD_GLASS_Clear();
			  				  for(int j = 0; j < LCD_DIGIT_MAX_NUMBER; j++) datatodisplay[i] = 0;
			  				  Gyro_Convert('TM', minute, datatodisplay);
			  				  BSP_LCD_GLASS_DisplayStrDeci(datatodisplay);
			  				  if(displaycase==displaycase_up)
			  				  {
			  					  displaycase=0;
			  					  minute--;
			  					  if(hour<0) minute=59;
			  					  HAL_Delay(50);
			  				  }
			  				  if(displaycase==displaycase_down)
			  				  {
			  					  displaycase=0;
			  					  minute++;
			  					  if(minute>23) minute=0;
			  					  HAL_Delay(50);
			  				  }
			  				  tm=10000*hour+100*minute+second;
			  			  }while(displaycase!=displaycase_center);
			  displaycase=0;
			  do
			  			  {
			  				  HAL_Delay(50);
			  				  BSP_LCD_GLASS_Clear();
			  				  for(int j = 0; j < LCD_DIGIT_MAX_NUMBER; j++) datatodisplay[i] = 0;
			  				  Gyro_Convert('TS', second, datatodisplay);
			  				  BSP_LCD_GLASS_DisplayStrDeci(datatodisplay);
			  				  if(displaycase==displaycase_up)
			  				  {
			  					  displaycase=0;
			  					  second--;
			  					  if(second<0) second=59;
			  					  HAL_Delay(50);
			  				  }
			  				  if(displaycase==displaycase_down)
			  				  {
			  					  displaycase=0;
			  					  second++;
			  					  if(second>23) second=0;
			  					  HAL_Delay(50);
			  				  }
			  				  tm=10000*hour+100*minute+second;
			  			  }while(displaycase!=displaycase_center);
			  RTC_TimeTypeDef timee;
			  timee.Hours=hour;
			  timee.Minutes=minute;
			  timee.Seconds=second;
			  HAL_RTC_SetTime(&hrtc,&timee,RTC_FORMAT_BIN);
			  displaycase=0;
			  cowyswietlic=0;
			  pierwszeuruchomienie=0;
		  }
		  displaycase=0;
		  flaginterrupt=0;
		  BSP_LCD_GLASS_Clear();
	  }
	  else if(flaginterrupt==2)//zapis do flasha tyle, ze nie do flasha
	  {
		  RTC_TimeTypeDef timee;
		  RTC_DateTypeDef datee;
		  HAL_RTC_GetTime(&hrtc, &timee, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &datee, RTC_FORMAT_BIN);
		  int timees=10000*timee.Hours+100*timee.Minutes+timee.Seconds;
		  if(liczbadanych==1000) liczbadanych=0;
		  if(acceleval!=0)
		  {
			  tablicadanych[liczbadanych]=acceleval;
			  tablicaczasow[liczbadanych]=timees;
			  liczbadanych++;
		  }
		  BSP_QSPI_Write(acceleval,N25Q128A_SUBSECTOR_SIZE*liczbadanych,2048);
		  BSP_QSPI_Write(timees,N25Q128A_SUBSECTOR_SIZE*liczbadanych+2048,2048);
		  HAL_Delay(1);
		  flaginterrupt=0;
	  }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
static void Gyro_Convert(uint8_t Axis, int32_t Value, uint16_t *DisplayString)
{
  uint8_t i = 0;
  uint32_t valuetoconvert = 0;

  valuetoconvert = (uint32_t) (ABS(Value));

  Convert_IntegerIntoChar(valuetoconvert, DisplayString);


	while(*(DisplayString + i) == '0')
	{
		  *(DisplayString + i) = ' ';
		  i++;
	}
	if (valuetoconvert==0) *(DisplayString+i-1)='0';
  /* Add Axis information 1st digit */
  *(DisplayString) = (uint16_t) Axis | DOUBLE_DOT;

  /* Handle negative value */
  if(Value < 0)
  {
    *(DisplayString + 1) = '-';
  }
}

void Compass_Calib(kompas *backup)
{
	int16_t MagBuffer[3];
	displaycase=0;

	  /* Display Info on LCD */
	  BSP_LCD_GLASS_Clear();
	  BSP_LCD_GLASS_ScrollSentence((uint8_t*)"      ROTATE BOARD ON ALL AXIS", 1, SCROLL_SPEED_HIGH);
	  BSP_LCD_GLASS_Clear();
	  BSP_LCD_GLASS_DisplayString((uint8_t*)"STOP");

	  /* Wait first measure */
	  HAL_Delay(25);

	  /* Loop to perform compass calibration */
	  do {
	    /* Get magnetometer values */
	    BSP_COMPASS_MagGetXYZ(MagBuffer);
	    /* Store maximum and minimum values */
	    backup->XmMax = max((int32_t) MagBuffer[0],backup->XmMax);
	    backup->XmMin = min((int32_t) MagBuffer[0],backup->XmMin);
	    backup->YmMax = max((int32_t) MagBuffer[1],backup->YmMax);
	    backup->YmMin = min((int32_t) MagBuffer[1],backup->YmMin);
	    backup->ZmMax = max((int32_t) MagBuffer[2],backup->ZmMax);
	    backup->ZmMin = min((int32_t) MagBuffer[2],backup->ZmMin);

	  } while(displaycase!=displaycase_center);
	  displaycase=0;
}

void Compass_Run(void)
{
	char  string_display[7];
	  displaycase=0;
	  do {
		  /* Read Gyroscope Angular data */
		      BSP_GYRO_GetXYZ(buffer);

		      /* Get absolute value */
		      xval = (int32_t) buffer[0] >>6;
		      yval = (int32_t) buffer[1] >>6;
		      zval = (int32_t) buffer[2] >>6;
		      xvalabs = ABS(xval);
		      yvalabs = ABS(yval);
		      zvalabs = ABS(zval);

	        sprintf(string_display," %3d",(int16_t) xval);
	        BSP_LCD_GLASS_Clear();
	        BSP_LCD_GLASS_DisplayString((uint8_t*)string_display);
	  } while(displaycase!=displaycase_center);
	  	  displaycase=0;
}

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
