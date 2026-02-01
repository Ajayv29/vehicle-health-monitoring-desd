
/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : FreeRTOS Sensor + Ultrasonic + CAN
 ******************************************************************************
 */
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "SEGGER_SYSVIEW.h"
#include "Disc_F407.h"
/* USER CODE END Includes */
/* USER CODE BEGIN PD */
#define SAMPLE 100
#define TRIG_PORT GPIOA
#define TRIG_PIN GPIO_PIN_1
#define ECHO_PORT GPIOA
#define ECHO_PIN GPIO_PIN_0
#define ADXL_ADDR (0x53 << 1)
#define STACK_SIZE 768
#define TOUCH1_PORT GPIOD
#define TOUCH1_PIN GPIO_PIN_2
#define TOUCH2_PORT GPIOD
#define TOUCH2_PIN GPIO_PIN_6
#define BUZZER_PORT GPIOE
#define BUZZER_PIN GPIO_PIN_3
/* USER CODE END PD */
/* USER CODE BEGIN PV */
SemaphoreHandle_t sensor_sem;
volatile float temp;
volatile float avg;
uint8_t rec_data[6];
int16_t x, y, z;
uint32_t distance_cm = 0;
uint8_t TxData[7];
volatile uint8_t touch1_state = 0;
volatile uint8_t touch2_state = 0;
volatile uint8_t touch_status = 0;
uint32_t TxMailbox;
CAN_TxHeaderTypeDef TxHeader;
/* USER CODE END PV */
/* USER CODE BEGIN PFP */
float temperature(void);
void vibration(void);
uint32_t ultrasonic_read_cm(void);
void data_collection(void *argument);
void data_send_to_esp(void *argument);
void ADXL_Init(I2C_HandleTypeDef *hi2c);
void ADXL_Read(I2C_HandleTypeDef *hi2c);
void CAN_Init(void);
void SystemClock_Config(void);
void Error_Handler(void);
void read_touch_sensors(void);
void buzzer_control(void);
/* USER CODE END PFP */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC3_Init();
    MX_I2C1_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_TIM2_Init();
    HAL_TIM_Base_Start(&htim2);
    SEGGER_SYSVIEW_Conf();
    sensor_sem = xSemaphoreCreateBinary();
    xSemaphoreGive(sensor_sem);
    CAN_Init();
    ADXL_Init(&hi2c1);
    xTaskCreate(data_collection, "SensorTask", STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(data_send_to_esp, "CANTask", STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
    while (1)
    {
    }
}
/* ================= SENSOR TASK ================= */
void data_collection(void *argument)
{
    while (1)
    {
        if (xSemaphoreTake(sensor_sem, portMAX_DELAY) == pdPASS)
        {
            SEGGER_SYSVIEW_PrintfHost("DATA_COLLECTION_TASK_RUNNING");
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
            temp = temperature();
            vibration();
            distance_cm = ultrasonic_read_cm();
            read_touch_sensors();
            buzzer_control();
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
            xSemaphoreGive(sensor_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}
/* ================= TEMPERATURE ================= */
float temperature(void)
{
    uint32_t sum = 0;
    for (int i = 0; i < SAMPLE; i++)
    {
        HAL_ADC_Start(&hadc3);
        HAL_ADC_PollForConversion(&hadc3, 10);
        sum += HAL_ADC_GetValue(&hadc3);
        HAL_ADC_Stop(&hadc3);
    }
    avg = (float)sum / SAMPLE;
    return (avg * 3.3f * 100.0f) / 4095.0f;
}
/* ================= ADXL ================= */
void vibration(void)
{
    ADXL_Read(&hi2c1);
}
void ADXL_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;
    HAL_I2C_Mem_Read(hi2c, ADXL_ADDR, 0x00, 1, &data, 1, 100);
    if (data != 0xE5)
        return;
    data = 0x0B;
    HAL_I2C_Mem_Write(hi2c, ADXL_ADDR, 0x31, 1, &data, 1, 100);
    data = 0x08;
    HAL_I2C_Mem_Write(hi2c, ADXL_ADDR, 0x2D, 1, &data, 1, 100);
}
void ADXL_Read(I2C_HandleTypeDef *hi2c)
{
    if (HAL_I2C_Mem_Read(hi2c, ADXL_ADDR, 0x32, 1, rec_data, 6, 10) != HAL_OK)
        return;
    x = (int16_t)((rec_data[1] << 8) | rec_data[0]);
    y = (int16_t)((rec_data[3] << 8) | rec_data[2]);
    z = (int16_t)((rec_data[5] << 8) | rec_data[4]);
}
/* ================= ULTRASONIC ================= */
uint32_t ultrasonic_read_cm(void)
{
    uint32_t timeout = 30000;
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < 10)
        ;
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_RESET)
        if (__HAL_TIM_GET_COUNTER(&htim2) > timeout)
            return 0;
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while (HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN) == GPIO_PIN_SET)
        if (__HAL_TIM_GET_COUNTER(&htim2) > timeout)
            return 0;
    uint32_t stop = __HAL_TIM_GET_COUNTER(&htim2);
    return (stop - start) / 58;
}
/* ================= CAN TASK ================= */
void data_send_to_esp(void *argument)
{
    while (1)
    {
        if (xSemaphoreTake(sensor_sem, portMAX_DELAY) == pdPASS)
        {
            SEGGER_SYSVIEW_PrintfHost("CAN_DATA_SEND_TO_ESP32_TASK_RUNNING");
            TxData[0] = (uint8_t)temp;
            TxData[1] = (distance_cm >> 8) & 0xFF;
            TxData[2] = distance_cm & 0xFF;
            TxData[3] = x & 0xFF;
            TxData[4] = y & 0xFF;
            TxData[5] = z & 0xFF;
            TxData[6] = touch_status;
            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0)
                HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
            xSemaphoreGive(sensor_sem);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
/* ================= CAN INIT ================= */
void CAN_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 14;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan2, &filter);
    TxHeader.StdId = 0x123;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;
    HAL_CAN_Start(&hcan2);
}
/* ================= SYSTEM CLOCK ================= */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 |
                                  RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
        Error_Handler();
}
/* ================= ERROR HANDLER ================= */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
        HAL_Delay(200);
    }
}
void read_touch_sensors(void)
{
    touch_status = 0;
    if (HAL_GPIO_ReadPin(TOUCH1_PORT, TOUCH1_PIN) == GPIO_PIN_SET)
        touch_status |= (1 << 0); // PD2 → bit0
    if (HAL_GPIO_ReadPin(TOUCH2_PORT, TOUCH2_PIN) == GPIO_PIN_SET)
        touch_status |= (1 << 0); // PD6 → bit1
    touch1_state = touch_status & 0x01;
    touch2_state = (touch_status >> 1) & 0x01;
}
void buzzer_control(void)
{
    if ((touch1_state == 1) || (touch2_state == 1) || distance_cm > 20)
    {
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET); // BUZZ
    }
    else
    {
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET); // SILENT
    }
}
