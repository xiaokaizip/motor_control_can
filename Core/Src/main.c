/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "math.h"
#include "printf.h"
#include "motor.h"
#include "pid.h"
#include "oled.h"
#include "retarget.h"
#include "menu.h"
#include "key.h"
#include "delay.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DECELERATION_RATIO_2006 36
#define DECELERATION_RATIO_3508 19.2032


uint16_t times = 0;
uint16_t times_can_recive = 0;
uint16_t times_can_send = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {

    if (hcan->Instance == CAN1) {
        if (times_can_recive > 200) {
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
            times_can_recive = 0;
        }
        CAN_RxHeaderTypeDef rx_header;
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, can_rx_data);
    }
}

pid_t_ pid_struct;
double set_speed = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    get_moto_measure(&moto_measure);

    pid_calc(&pid_struct, (float) moto_measure.speed_rpm, set_speed);
    set_moto_current((int16_t) pid_struct.pos_out);
    times++;
    times_can_recive++;
    times_can_send++;
}

void can_init(void) {
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = (CAN_ID) << 5;
    can_filter_st.FilterIdLow = 0;
    can_filter_st.FilterMaskIdHigh = 0xFFE0;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 1;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan, &can_filter_st);
    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
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
    MX_CAN_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */

    RetargetInit(&huart2);

//    OLED_Init();                           //OLED初始
//    OLED_Clear();                         //清屏

    can_init();
    HAL_TIM_Base_Start_IT(&htim1);
    bool key_state = true;
    HAL_UART_Transmit(&huart2, "start\r\n", 7, HAL_MAX_DELAY);
    //位置式pid
    PID_struct_init(&pid_struct, POSITION_PID, 20000, 20000, 30.0f, 0.025f, 13.0f);
    //增量式pid
    //PID_struct_init(&pid_struct, DELTA_PID, 20000, 20000, 10.0f, 0.1f, 10.0f);

    pid_struct.max_err = 20000.0f;
    uint16_t max_speed_change = 5000;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    bool flag = true;
    bool start_flag = false;
    float dt = 0.002f;
    float t = 0;
    double a = 1;
    double w = 1.884;
    double b = 2.090 - a;
    double set_speed_temp = 864 * 2;

    struct motor_oled par = {
            0, 1, 1, 10.0f
    };


    //前期的参数设置

    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */


        // OLED_Clear();
        //set_speed = 10 * DECELERATION_RATIO_3508 * 3;
        set_speed = 90 * a * (sin(w * 0.001 * times) + b) * DECELERATION_RATIO_3508 / 3.1415926;
        pid_calc(&pid_struct, (float) moto_measure.speed_rpm, set_speed);
        printf("%f,%d\n", set_speed, moto_measure.speed_rpm);

        // OLED_ShowNum(8 * 7, 0, set_speed, 4, 16, 1);
        //delayus(1000);


    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
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
