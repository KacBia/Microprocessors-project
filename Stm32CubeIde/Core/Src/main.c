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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc522.h"
#include "servo.h"
#define ADDRESS_LCD 0x27
#include "keypad.h"
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
/**
  * @brief  1. Declares and initializes variables for controlling a servo motor,
  *  2. handling user input,
  *  3. tracking time,
  *  4. managing distance measurements,
  *  5.ensuring RFID validation in the system.
  * @param  None
  * @retval None
  */
SERVO_Handle_TypeDef hservo1 = {.PwmOut = PWM_INIT_HANDLE(&htim9, TIM_CHANNEL_1) };
uint8_t status;
uint8_t str[MAX_LEN];
uint8_t sNum[5];
GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  //< cm
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
char userInput[5] = {0}; //< Buffer for user input
uint8_t pinIndex = 0;    //< Tracks number of characters entered on keypad
uint8_t targetDistance = 0; //< Target distance input from user
uint8_t isRFIDValidated = 0; //< Flag to ensure RFID is validated first
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief LCD Initialization and Communication Functions.
  * @note This code provides basic functions to interface with an LCD over I2C.
  *       The functions include sending commands and data to the LCD,
  *       initializing the LCD, and printing a string to the display.
  * @details
  *       - lcd_send_cmd: Sends a command byte to the LCD.
  *       - lcd_send_data: Sends a data byte (character) to the LCD.
  *       - lcd_init: Initializes the LCD with specific settings for 4-bit mode.
  *       - lcd_send_string: Sends a null-terminated string to the LCD.
  *       These functions use the HAL I2C API to communicate with the LCD.
  * @note The LCD address is defined as ADDRESS_LCD and needs to match the
  *       LCD module's address in the circuit. The initialization sequence
  *       must be configured for the specific LCD being used.
  * @retval None
  */
/* USER CODE END 0 */
void lcd_send_cmd (char cmd)
{
	char data_u, data_l,ret1;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;
	data_t[1] = data_u|0x08;
	data_t[2] = data_l|0x0C;
	data_t[3] = data_l|0x08;
	ret1 = HAL_I2C_Master_Transmit (&hi2c1, (ADDRESS_LCD << 1) ,(uint8_t *) data_t, 4, 100);
}
void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;
	data_t[1] = data_u|0x09;
	data_t[2] = data_l|0x0D;
	data_t[3] = data_l|0x09;
	HAL_I2C_Master_Transmit (&hi2c1, (ADDRESS_LCD << 1),(uint8_t *) data_t, 4, 100);
}
void lcd_init (void)
{
	lcd_send_cmd (0x28);
	HAL_Delay(1);
	lcd_send_cmd (0x08);
	HAL_Delay(1);
	lcd_send_cmd (0x01);
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd (0x06);
	HAL_Delay(1);
	lcd_send_cmd (0x0C);
}
void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  * @note   This function serves as the starting point of the application.
  *         It performs the following steps:
  *         1. Initializes the HAL library to reset peripherals, configure the
  *            Flash interface, and the SysTick timer.
  *         2. Configures the system clock via SystemClock_Config().
  *         3. Initializes all configured peripherals such as GPIO, USART3,
  *            USB OTG, SPI1, TIM1, TIM2, I2C1, and TIM9.
  *         4. Performs application-specific initialization, including:
  *            - Initializing the MFRC522 RFID module.
  *            - Starting the TIM1 base timer.
  *            - Configuring the TRIG GPIO pin to RESET state.
  *            - Setting up a servo motor using SERVO_Init() and
  *              setting its initial position to 0 degrees.
  *            - Initializing a keypad handler for a 4x4 keypad matrix.
  * @note   The application functionality is implemented after the peripheral
  *         and application-specific initialization in the subsequent sections.
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin,RESET);
  SERVO_Init(&hservo1);
  SERVO_WritePosition(&hservo1, 0.0f);
  char uart_buffer[100];
  char pressed;
  KEYPAD_Handle_TypeDef hkeypad = KEYPAD_4x4_INIT_HANDLE(KEYPAD);
  float servo_position = 0.0f;
  uint8_t direction = 1; //<1 for increasing angle, 0 for decreasing angle
//< Declare key array with space for 2 characters + null terminator
  char key[3] = {'\0', '\0', '\0'};  //<Initialize with null characters
  static uint8_t input_index = 0;
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /**
    * @brief  Main loop responsible for RFID chip scanning,
    *  user input handling to set a target distance, and controlling
    *  the Servo motor based on distance measured by an Ultra-sonic sensor.
    * @param:None
    * This loop continuously performs the following tasks:
    * - Initializes the LCD and prompts the user to scan an RFID chip.
    * - Requests the RFID chip and handles any anti-collision protocol.
    * - If the correct RFID chip is detected, prompts the user to input a target distance.
    * - Waits for the user to enter a two-digit target distance via a keypad.
    * - Displays the target distance on the LCD.
    * - Controls a Servo motor to move continuously based on the set distance,
    * adjusting its position in increments.
    * - Measures the distance using an Ultra-sonic sensor.
    * - Compares the measured distance to the target distance.
    * - If the measured distance matches the target distance,
    * stops the Servo, and displays a confirmation message on the LCD.
    * - Repeats this process indefinitely until terminated.
    * @note:   The RFID detection, user input handling, Servo control,
    * and Ultra-sonic distance measurement processes are executed
    * within nested infinite loops for continuous operation.
    * @retval: None
    */
while (1) {
     lcd_init();
     lcd_send_cmd(0x80); //< 0x80 is to display on first line
     lcd_send_string("Scan Chip!");
     HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,SET);
     HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,RESET);
     HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,RESET);
     //< Request and anti-collision for RFID chip detection
     status = MFRC522_Request(PICC_REQIDL, str); //< Request RFID
     status = MFRC522_Anticoll(str); //< Anti-collision
     memcpy(sNum, str, 5); //< Copy chip data
     sprintf(uart_buffer, "Chip Number: %02X %02X %02X %02X %02X\r\n", str[0], str[1], str[2], str[3], str[4]);
     HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
     //< Check for correct chip (replace with correct chip ID)
     if ((str[0] == 240) && (str[1] == 12) && (str[2] == 77) && (str[3] == 116) && (str[4] == 197)) {
         lcd_init(); //< Display prompt to enter position and clear the one before
         lcd_send_cmd(0x80);
         lcd_send_string("Enter Pos:");
        //< Wait for user input (2 digits)
         input_index = 0;  //< Reset input index
     while (input_index < 2) {
         pressed = KEYPAD_GetKey(&hkeypad, 0);//< Get user input
       if (pressed != '\0') {
         key[input_index] = pressed;  //< Store the pressed key
         input_index++;  //< Move to the next index for the second number
         }
         }
         key[input_index] = '\0';  // Null-terminate the string
         targetDistance = atoi(key);  // Convert input string to an integer for the distance
         //< Display target distance on the LCD
       lcd_init();
       lcd_send_cmd(0x80);
       lcd_send_string("Target Distance: ");
       lcd_send_cmd(0xc0);
       lcd_send_string(key);  //< Display the target distance
       sprintf(uart_buffer, "Target Distance Set: %d cm\r\n", targetDistance);
       HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
       HAL_Delay(1000);  //< Delay to show target distance
             // Begin moving the servo and checking distance
     while (1) {  //< Infinite loop for continuous rotation
       if (direction) {
          servo_position += 5.0f;
        if (servo_position >= 180.0f) {
          servo_position = 180.0f;
           direction = 0;
        }
         } else {
              servo_position -= 5.0f;
            if (servo_position <= 0.0f) {
             servo_position = 0.0f;
               direction = 1;
            }
              }
      //< Move the servo to the current position
         SERVO_WritePosition(&hservo1, servo_position);
         HAL_Delay(100);  // Adjust delay for smoother movement
      //< Measure distance using the ultrasonic sensor
         HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, SET);
          __HAL_TIM_SET_COUNTER(&htim1, 0);
          while (__HAL_TIM_GET_COUNTER (&htim1) < 10);
          HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, RESET);
          pMillis = HAL_GetTick();
          while (!(HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && pMillis + 10 >  HAL_GetTick());
          Value1 = __HAL_TIM_GET_COUNTER (&htim1);
          pMillis = HAL_GetTick();
          while ((HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && pMillis + 50 > HAL_GetTick());
          Value2 = __HAL_TIM_GET_COUNTER (&htim1);
          Distance = (Value2-Value1)* 0.034/2;
          HAL_Delay(50); //< input capture that will use
          sprintf(uart_buffer, "Current Distance: %d cm\r\n", Distance);
          HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
         //< Check if distance matches target
         if (Distance == targetDistance) {
         SERVO_WritePosition(&hservo1, servo_position);  //< Stop the servo when target distance is detected
         lcd_init();
         lcd_send_cmd(0x80);
         lcd_send_string("Detected");  //< Display "Detected" on LCD to alert
         HAL_GPIO_WritePin(GREEN_LED_GPIO_Port,GREEN_LED_Pin,SET);
         HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,SET);
         HAL_GPIO_WritePin(RED_LED_GPIO_Port,RED_LED_Pin,RESET);
         sprintf(uart_buffer, "Detection Successful: %d cm matched\r\n", Distance);
         HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
         HAL_Delay(3000);
        break;  //< Exit the loop once the target distance is detected
      }
        }
         }
        HAL_UART_Transmit(&huart3, (uint8_t*)&Distance, 1, 10);

         HAL_Delay(100);  //< Adjust delay for smoother operation
   }
}
  /* USER CODE END 3 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
