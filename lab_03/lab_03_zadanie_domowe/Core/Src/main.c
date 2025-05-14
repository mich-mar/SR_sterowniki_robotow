/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> // Dodanie nagłówka do obsługi printf
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* TIM1 (Input Capture) */
/* Częstotliwość timera TIM1:
   f_TIM1 = SystemCoreClock / ((prescaler_1 + 1) * (ARR_1 + 1))
   f_TIM1 = 72,000,000 / ((7200 + 1) * (1000 + 1))
   f_TIM1 = 72,000,000 / (7201 * 1001)
   f_TIM1 ≈ 9.97 Hz

   Okres timera TIM1:
   T_TIM1 = 1 / f_TIM1
   T_TIM1 ≈ 1 / 9.97 ≈ 0.1003 sekundy

   Rozdzielczość timera 1:
   Tick = (prescaler_1 + 1) / SystemCoreClock = 7201 / 72,000,000 = 100.01 μs
*/
#define prescaler_1 7200 // Preskaler dla timera 1 (pomiar impulsu)
#define ARR_1 1000       // Rejestr ARR timera 1, okres pomiaru

/* TIM3 (PWM) */
/* Częstotliwość timera TIM3:
   f_TIM3 = SystemCoreClock / ((prescaler_3 + 1) * (ARR_3 + 1))
   f_TIM3 = 72,000,000 / ((720 + 1) * (4000 + 1))
   f_TIM3 = 72,000,000 / (721 * 4001)
   f_TIM3 ≈ 24.99 Hz

   Okres timera TIM3:
   T_TIM3 = 1 / f_TIM3
   T_TIM3 ≈ 1 / 24.99 ≈ 0.040 sekundy

   Okres sygnału PWM w ms:
   T_PWM = (ARR_3 + 1) * (prescaler_3 + 1) / (72,000,000 / 1000) = 4001 * 721 / 72000 = 40.01 ms
*/
#define prescaler_3 720 // Preskaler dla timera 3 (PWM)
#define ARR_3 4000      // Rejestr ARR timera 3, okres PWM

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// Zmienne globalne do przechowywania pomiarów
volatile uint32_t risingEdgeTime = 0;           // Czas zbocza narastającego
volatile uint32_t fallingEdgeTime = 0;          // Czas zbocza opadającego
volatile uint32_t overflowCount = 0;            // Licznik przepełnień timera
volatile uint32_t overflowsRisingToFalling = 0; // Liczba przepełnień między zboczem narastającym a opadającym
volatile uint32_t pulseDuration = 0;            // Czas trwania impulsu w cyklach timera
volatile uint8_t risingEdgeDetected = 0;        // Flaga wykrycia zbocza narastającego
volatile uint8_t measurementComplete = 0;       // Flaga zakończenia pomiaru

// Zmienne do obliczeń
volatile float pwmDutyCycle = 0.0;      // Wypełnienie PWM w %
volatile float pulseDurationMs = 0.0;   // Czas trwania impulsu w ms
volatile float pwmPeriodMs = 0.0;       // Okres sygnału PWM w ms
volatile uint8_t inputCaptureState = 0; // Stan pomiaru Input Capture (0-początek, 1-zbocze narastające, 2-zbocze opadające)

// Zmienne do wyświetlania i zmiany wypełnienia
static uint32_t lastTime_pwm = 0, lastTime_IC = 0;
static uint8_t pwmState = 0;
uint32_t currentTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Callback wywoływany przy rejestracji impulsu na kanale 1 (np. TIM1 CH1)
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      // Timer 1 jest w trybie Input Capture w trybie Direct Mode -
      // zbocza narastające i opadające są rejestrowane na tym samym kanale

      // Sprawdzamy aktualny poziom sygnału, aby ustalić czy to zbocze narastające czy opadające
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)
      {
        // Zbocze narastające
        risingEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        overflowsRisingToFalling = 0;
        risingEdgeDetected = 1;
      }
      else
      {
        // Zbocze opadające (tylko jeśli wcześniej było narastające)
        if (risingEdgeDetected)
        {
          fallingEdgeTime = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

          // Obliczanie czasu trwania impulsu
          if (fallingEdgeTime >= risingEdgeTime)
          {
            // Normalny przypadek - bez przepełnienia licznika między pomiarami
            pulseDuration = fallingEdgeTime - risingEdgeTime;
          }
          else
          {
            // Przypadek z przepełnieniem licznika między pomiarami
            pulseDuration = ((ARR_1 + 1) - risingEdgeTime) + fallingEdgeTime;
          }

          // Dodanie przepełnień licznika które wystąpiły między zboczami
          pulseDuration += overflowsRisingToFalling * (ARR_1 + 1);

          // Resetowanie flagi zbocza narastającego i ustawienie flagi zakończenia pomiaru
          risingEdgeDetected = 0;
          measurementComplete = 1;
        }
      }
    }
  }
}

// Callback wywoływany przy przepełnieniu licznika (np. TIM1 Update)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    overflowCount++;

    // Jeśli jesteśmy w trakcie pomiaru (po zboczu narastającym, przed opadającym)
    if (risingEdgeDetected)
    {
      overflowsRisingToFalling++;
    }
  }
}

// Przekierowanie printf do UART
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY); 
  return len;                                              
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  // Uruchomienie Input Capture z przerwaniami
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);

  // Uruchomienie generatora PWM (TIM3)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Włączenie przerwania od przepełnienia timera 1
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

  // Ustawienie początkowego wypełnienia PWM na 10%
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (htim3.Instance->ARR * 10) / 100);

  // Komunikat inicjalizacyjny
  printf("System uruchomiony. PWM na TIM3, pomiar szerokosci impulsu na TIM1\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    currentTime = HAL_GetTick();

    if (currentTime - lastTime_IC >= 1000)
    {
      lastTime_IC = currentTime;

      if (measurementComplete)
      {
        // Obliczenie czasu trwania impulsu w milisekundach
        // Wzór: czas [ms] = (licznik * (prescaler+1)) / (częstotliwość zegara [Hz] / 1000)
        float tickTime_us = ((prescaler_1 + 1) * 1000000.0) / SystemCoreClock; // Czas jednego tiku w mikrosekundach
        pulseDurationMs = (pulseDuration * tickTime_us) / 1000.0;              // Konwersja us na ms

        // Obliczenie okresu sygnału PWM na TIM3 (teoretycznie)
        // Okres = (ARR_3 + 1) * (prescaler_3 + 1) / SystemCoreClock
        pwmPeriodMs = ((float)(ARR_3 + 1) * (prescaler_3 + 1)) / (SystemCoreClock / 1000.0);

        // Obliczenie wypełnienia (duty cycle) PWM
        pwmDutyCycle = (pulseDurationMs / pwmPeriodMs) * 100.0;

        // Wyświetlenie wyników
        printf("Czas impulsu: %.3f ms, Okres PWM: %.3f ms, Wypelnienie: %.2f%%\r\n",
               pulseDurationMs, pwmPeriodMs, pwmDutyCycle);

        // Resetowanie flagi zakończenia pomiaru
        measurementComplete = 0;
      }
      else
      {
        printf("Oczekiwanie na pomiar...\r\n");
      }
    }

    // Zmiana wypełnienia PWM co 2 sekundy
    if (currentTime - lastTime_pwm >= 2000)
    {
      lastTime_pwm = currentTime;

      // Zmiana wypełnienia PWM
      switch (pwmState)
      {
      case 0:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (ARR_3 * 10) / 100); // 10%
        printf("Ustawiono wypelnienie PWM: 10%%\r\n");
        pwmState = 1;
        break;
      case 1:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (ARR_3 * 40) / 100); // 40%
        printf("Ustawiono wypelnienie PWM: 40%%\r\n");
        pwmState = 2;
        break;
      case 2:
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (ARR_3 * 80) / 100); // 80%
        printf("Ustawiono wypelnienie PWM: 80%%\r\n");
        pwmState = 0;
        break;
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq(); // Zatrzymanie przerwań
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
