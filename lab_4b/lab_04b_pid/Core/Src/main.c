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
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile int adc_flag;        // Flaga sygnalizująca zakończenie konwersji ADC
volatile int adc_value;       // Aktualny odczyt ADC (wartość mierzona)
volatile int dac_value;       // Aktualna wartość wyjściowa DAC (setpoint z sinusoidy)
volatile int dac_control;     // Wartość wyjściowa regulatora PID
volatile uint16_t dac_index;  // Aktualny indeks w tablicy lookup sinusoidy
volatile uint8_t dac_nperiod; // Licznik do generowania sinusoidy z odpowiednią prędkością
#define dac_nperiod_max 40   // Maksymalna wartość licznika przed przejściem do następnego punktu sinusoidy
float V_REF = 3.3f;

// Parametry sinusoidy w zakresie 0.4V - 2.9V
#define V_MIN 0.4f          // Minimalne napięcie sinusoidy [V]
#define V_MAX 2.9f          // Maksymalne napięcie sinusoidy [V]
#define V_AMPLITUDE ((V_MAX - V_MIN) / 2.0f)  // Amplituda sinusoidy [V]
#define V_OFFSET (V_MIN + V_AMPLITUDE)         // Offset sinusoidy [V]

cpid_t pid; // Instancja regulatora PID
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t sin_wave[] = {
    2048, 2112, 2176, 2239, 2302, 2364, 2424, 2483, 2541,
    2596, 2649, 2700, 2748, 2794, 2836, 2876, 2912, 2945,
    2974, 2999, 3021, 3039, 3053, 3063, 3069, 3071, 3069,
    3063, 3053, 3039, 3021, 2999, 2974, 2945, 2912, 2876,
    2836, 2794, 2748, 2700, 2649, 2596, 2541, 2483, 2424,
    2364, 2302, 2239, 2176, 2112, 2048, 1983, 1919, 1856,
    1793, 1731, 1671, 1612, 1554, 1499, 1446, 1395, 1347,
    1301, 1259, 1219, 1183, 1150, 1121, 1096, 1074, 1056,
    1042, 1032, 1026, 1024, 1026, 1032, 1042, 1056, 1074,
    1096, 1121, 1150, 1183, 1219, 1259, 1301, 1347, 1395,
    1446, 1499, 1554, 1612, 1671, 1731, 1793, 1856, 1919, 1983};

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == &hadc1)
  {
    adc_flag = 1;
    adc_value = HAL_ADC_GetValue(&hadc1);
  }
}

/* Funkcja przeliczająca surową wartość DAC na napięcie*/
float raw_to_voltage(int raw_value)
{
  return (raw_value / 4095.0f) * V_REF;
}

/* Funkcja przeliczająca napięcie na surową wartość DAC */
int voltage_to_raw(float voltage)
{
  return (int)((voltage / V_REF) * 4095.0f);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim7)
  {
    uint16_t original_sine = sin_wave[dac_index];
    
    float normalized = (float)(original_sine - 1024) / (3071 - 1024);
    
    uint16_t v_min_raw = voltage_to_raw(0.4f);   // 0.4V w jednostkach DAC
    uint16_t v_max_raw = voltage_to_raw(2.9f);  // 2.9V w jednostkach DAC
    
    dac_value = v_min_raw + (uint16_t)(normalized * (v_max_raw - v_min_raw));

    ++dac_nperiod;
    if (dac_nperiod >= dac_nperiod_max)
    {
      dac_nperiod = 0;
      ++dac_index;
      if (dac_index >= 100)
        dac_index = 0;
    }
  }
}

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
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  // Konfiguracja i uruchomienie DAC
  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); // Ustaw początkowe wyjście DAC na 0
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);                        // Uruchom kanał 1 DAC

  // Konfiguracja i uruchomienie timerów
  HAL_TIM_Base_Start(&htim6);    // Uruchom TIM6 (baza czasowa PID, bez przerwań)
  HAL_TIM_Base_Start_IT(&htim7); // Uruchom TIM7 z przerwaniami (generator sinusoidy)

  // Konfiguracja i uruchomienie ADC
  HAL_ADC_Start_IT(&hadc1); // Uruchom ADC1 w trybie przerwań

  // Inicjalizacja regulatora PID
  // Parametry: P=1.0, I=0.0, D=0.0, 10 bitów ułamkowych, czas próbkowania 1ms
  pid_init(&pid, 1.0f, 0.6f, 0.05f, 10, 1);

  // Ustawienie ograniczeń składowych PID (ochrona anti-windup i nasycenia)
  pid.p_max = pid_scale(&pid, 4095);     // Maks. wyjście proporcjonalne = pełny zakres DAC
  pid.p_min = pid_scale(&pid, -4095);    // Min. wyjście proporcjonalne = ujemny pełny zakres DAC
  pid.i_max = pid_scale(&pid, 4095);     // Maks. wyjście całkujące = pełny zakres DAC
  pid.i_min = pid_scale(&pid, -4095);    // Min. wyjście całkujące = ujemny pełny zakres DAC
  pid.d_max = pid_scale(&pid, 4095);     // Maks. wyjście różniczkujące = pełny zakres DAC
  pid.d_min = pid_scale(&pid, -4095);    // Min. wyjście różniczkujące = ujemny pełny zakres DAC
  pid.total_max = pid_scale(&pid, 4095); // Maks. całkowite wyjście PID = pełny zakres DAC
  pid.total_min = pid_scale(&pid, 0);    // Min. całkowite wyjście PID = 0 (DAC nie może być ujemny)

  uint16_t ctr = 0; // Licznik do wyświetlania wartości

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    if (adc_flag == 1) // Sprawdź czy gotowa jest nowa konwersja ADC
    {
      ctr++; // Inkremntowanie wartości do wyświetlania
      adc_flag = 0; // Wyczyść flagę gotowości ADC

      // Wykonaj algorytm sterowania PID
      // Wejście: adc_value (mierzona/sprzężenie zwrotne), dac_value (setpoint z sinusoidy)
      // Wyjście: dac_control (sygnał sterujący do aktuatora)
      dac_control = pid_calc(&pid, adc_value, dac_value);

      // Zastosuj wyjście sterowania do DAC
      HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_control);

      // Logowanie danych przez UART
      // Format: "wartość_mierzona;setpoint;wyjście_sterowania;"
      // Pozwala to na monitorowanie i analizę wydajności systemu w czasie rzeczywistym
      if (ctr >= 100)
      {
        // printf("275417;%d;%d;%d;\r\n", adc_value, dac_value, dac_control);
        printf("275417;%.2f;%.2f;%.2f;\r\n", raw_to_voltage(adc_value), raw_to_voltage(dac_value), raw_to_voltage(dac_control));
        ctr = 0;
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
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
