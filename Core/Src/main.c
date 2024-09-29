/* USER CODE BEGIN Header */
/**
******************************************************************************
* @file           : main.c
* @brief          : Main program body
******************************************************************************
* @attention
*
* Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"
//#include "dma.h"

#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DATA_SIZE 524288000 //500*1024*1024
#define BLOCK_SIZE         512 // 一个块的字节数
#define DATA_SIZE_TO_WRITE 10*1024*1024 // 1MB = 1*1024*1024字节
#define DMA_NUM_BLOCKS_TO_WRITE 8 // 每一次DMA写入块的数量
#define DMA_NUM_BLOCKS_TO_READ  1 // 每一次DMA读出块的数量
#define NUM_TIMES_TO_WRITE DATA_SIZE_TO_WRITE/ (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_WRITE) // 总共要写入的次数
#define NUM_TIMES_TO_READ  DATA_SIZE_TO_WRITE/ (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_READ) // 总共要写入的次数

#define BUFFER_SIZE         16384         // 缓冲区大小

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t buffer1[BUFFER_SIZE];
//uint8_t buffer2[BUFFER_SIZE];
//uint8_t *active_buffer;  // 当前活跃缓冲区
//uint8_t *processing_buffer;  // 正在处理的缓冲区

__ALIGN_BEGIN uint8_t buffer_TX[BUFFER_SIZE] __ALIGN_END;  // 4字节对齐，发送缓冲区
__ALIGN_BEGIN uint8_t buffer_RX[BLOCK_SIZE*DMA_NUM_BLOCKS_TO_READ] __ALIGN_END;  // 4字节对齐，接收缓冲区

volatile uint8_t sd_write_complete = 0; // 写入完成标志
volatile uint8_t sd_read_complete = 0;  // 读取完成标志

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* DMA回调函数 */
void HAL_SD_WriteCpltCallback(SD_HandleTypeDef *hsd)
{
    sd_write_complete = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
//    printf("SDIO 传输错误！\n");
    // 添加错误处理逻辑，如重试或复位 SDIO
}

void generate_increasing_numbers(unsigned char *array, int num_bytes, int byte_size) {
    // 确保byte_size是1, 2, 3或4字节的递增数
    if (byte_size < 1 || byte_size > 4) {
        printf("递增数的字节数必须是1, 2, 3或4\n");
        return;
    }

    // 确保num_bytes是byte_size的倍数
    if (num_bytes % byte_size != 0) {
        printf("输入的字节数必须是%d的倍数\n", byte_size);
        return;
    }

    int num_values = num_bytes / byte_size;  // 需要生成的递增数的数量

    for (int i = 0; i < num_values; i++) {
        unsigned int value = i;  // 递增数从0开始

        // 根据byte_size将value的每个字节按照低字节在前的顺序放入数组
        for (int j = 0; j < byte_size; j++) {
            array[i * byte_size + j] = (value >> (8 * j)) & 0xFF;
        }
    }
}
int compare_buffers(const unsigned char *buffer1, const unsigned char *buffer2, int size) {
    for (int i = 0; i < size; i++) {
        if (*(buffer1+i) != *(buffer2+i)) {
            return i;  // 返回第一个不匹配的字节索引
        }
    }
    return -1;  // 如果完全相同，返回-1
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
//    // DMA 读取完成时切换缓冲区
//    uint8_t *temp = active_buffer;
//    active_buffer = processing_buffer;
//    processing_buffer = temp;
}

void process_data(uint8_t* buffer, uint32_t block) {
    // 在这里处理数据，例如将数据写入其他存储器，或进行计算
    static int mismatch_index;
    mismatch_index = compare_buffers(buffer_TX+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ)%BUFFER_SIZE, buffer, BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ);
    if (mismatch_index != -1) {
        printf("数据验证失败在块 %d\r\n", block);
//        while (1); // 进入死循环，数据错误
    }
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
  HAL_Delay(100);
    
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SDIO_SD_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
    
    
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\r\n");
    printf("\r\n");
    printf("\r\n");
    printf("Start testing \n");
    
    uint32_t sys_clk = HAL_RCC_GetSysClockFreq();
    printf("System Clock Frequency: %d Hz\n", sys_clk);
    RCC_ClkInitTypeDef clkinitstruct = {0};
    RCC_OscInitTypeDef oscinitstruct = {0};
    uint32_t flashLatency;
    /* Get clock configuration */
    HAL_RCC_GetClockConfig(&clkinitstruct, &flashLatency);
    /* Get oscillator configuration */
    HAL_RCC_GetOscConfig(&oscinitstruct);
    /* Assuming PLL is used as clock source and PLLQ is configured */
    uint32_t pll_source_freq = (oscinitstruct.PLL.PLLSource == RCC_PLLSOURCE_HSE) ? HSE_VALUE : HSI_VALUE;
    uint32_t pll_vco_freq = (pll_source_freq / oscinitstruct.PLL.PLLM) * oscinitstruct.PLL.PLLN;
    uint32_t sdio_clk = pll_vco_freq / oscinitstruct.PLL.PLLQ;
    printf("SDIO Clock Frequency: %d Hz\n", sdio_clk / (hsd.Init.ClockDiv + 2));
    
    // 确保 SD 卡已初始化成功
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        printf("SD 卡未准备好，进入死循环。\n");
        while (1);
    }
    
    printf("SD 卡初始化成功！\n");
    
    uint32_t BlockAdd = 0; // 起始块号
    
    /*--------------------------SD卡写测试----------------------------------*/
    
#define MAX_RETRIES 3          // 最大重试次数
#define RETRY_DELAY_MS 1      // 重试前的延迟（毫秒）
    
    generate_increasing_numbers(buffer_TX, sizeof(buffer_TX), 4) ;
    
    // 记录写入开始时间
    printf("开始写入数据到 SD 卡...\n");
    uint32_t start_time = HAL_GetTick();

    for (uint32_t block = 0; block < NUM_TIMES_TO_WRITE; block++) {
        uint8_t retry = 0;
        HAL_StatusTypeDef status;
        
        // 重试写入操作
        do {
            // 写入一个块
            status = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t*)buffer_TX+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_WRITE)%BUFFER_SIZE, BlockAdd, DMA_NUM_BLOCKS_TO_WRITE);
//            status = HAL_SD_WriteBlocks(&hsd, (uint8_t*)buffer_TX+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_WRITE)%BUFFER_SIZE, BlockAdd, DMA_NUM_BLOCKS_TO_WRITE,10);
            if (status == HAL_OK) {
                // 等待写入完成
                while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {}
                BlockAdd += DMA_NUM_BLOCKS_TO_WRITE; // 更新块号以写入下一个块
                break; // 写入成功，退出重试循环
            } else {
                printf("写入块 %d 失败，状态：%d。重试 %d/%d\r\n", 
                       BlockAdd, HAL_SD_GetError(&hsd), retry + 1, MAX_RETRIES);
                retry++;
                HAL_Delay(RETRY_DELAY_MS); // 等待一段时间再重试
            }
        } while (retry < MAX_RETRIES);
        
        // 如果达到最大重试次数仍未成功，记录错误并继续
        if (status != HAL_OK) {
            printf("写入块 %d 失败，达到最大重试次数，跳过此块。\n", BlockAdd);
            BlockAdd += DMA_NUM_BLOCKS_TO_WRITE; // 更新块号，避免无限循环
            continue; // 继续写入下一个块
        }
        
        // 每写入10000块打印一次进度
        if ((block + 1) % 10000 == 0) {
            printf("已写入 %d MB...\r\n", ((block + 1) * (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_WRITE)) / (1024 * 1024));
        }

    }
    
    // 记录写入结束时间
    uint32_t end_time = HAL_GetTick();
    
    // 计算总写入时间（秒）
    float elapsed_time_sec = (end_time - start_time) / 1000.0f;
    
    // 计算写入速度（MB/s）
    float write_speed = DATA_SIZE_TO_WRITE / (1024.0f * 1024.0f) / elapsed_time_sec;
    
    printf("数据写入完成！\n");
    printf("总写入时间: %.2f 秒\n", elapsed_time_sec);
    printf("写入速度: %.4f MB/s\n", write_speed);
    
    /*--------------------------SD卡读取速度测试（可选）--------------------------*/
    // 记录读取开始时间
    printf("开始读取数据从 SD 卡...\n");

    start_time = HAL_GetTick();
    for (uint32_t block = 0; block < NUM_TIMES_TO_READ; block++) 
    {
        // 启动 DMA 读取数据到当前活跃缓冲区
        if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)buffer_RX, block*DMA_NUM_BLOCKS_TO_READ, DMA_NUM_BLOCKS_TO_READ) == HAL_OK) {
            // 等待读取完成
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER);
        } 
        else {
            printf("读取块 %d 失败，状态：%d\r\n", block, HAL_SD_GetError(&hsd));
//            while (1);  // 进入死循环，读取错误
        }
        
        if (block % 10000 == 0 && block != 0) {
            printf("已读取 %d MB...\r\n", (block * BLOCK_SIZE) / (1024 * 1024));
        }
    }
    // 记录读取结束时间
    end_time = HAL_GetTick();
    
    // 计算总读取时间（秒）
    elapsed_time_sec = (end_time - start_time) / 1000.0f;
    
    // 计算读取速度（MB/s）
    write_speed = DATA_SIZE_TO_WRITE / (1024.0f * 1024.0f) / elapsed_time_sec;
    
    printf("数据读取完成！\n");
    printf("总读取时间: %.2f 秒\n", elapsed_time_sec);
    printf("读取速度: %.2f MB/s\n", write_speed);
    
    /*--------------------------SD卡读测试（可选）--------------------------*/
    printf("开始读取数据从 SD 卡...\n");
    
//    generate_increasing_numbers(buffer_TX, sizeof(buffer_TX), 2) ;
    
    for (uint32_t block = 0; block < NUM_TIMES_TO_READ; block++) {
        // 读取数据从 SD 卡
        if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)buffer_RX, block*DMA_NUM_BLOCKS_TO_READ, DMA_NUM_BLOCKS_TO_READ) == HAL_OK) {
            // 等待读取完成
//            printf("等待读取完成 %d\r\n", block);

            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {}
            // 验证数据
//            printf("验证数据 %d\r\n", block);

            static int mismatch_index;
            mismatch_index = compare_buffers(buffer_TX+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ)%BUFFER_SIZE, buffer_RX, BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ);
            if (mismatch_index != -1) {
                printf("数据验证失败在块 %d\r\n", block);
                while (1); // 进入死循环，数据错误
            }
            else
            {   
//                printf("数据读取和验证完成块 %d\r\n", block);
            }
            
        } else {
            printf("读取块 %d 失败，状态：%d\r\n", block, HAL_SD_GetError(&hsd));
            while (1); // 进入死循环，读取错误
        }
        
        if (block % 10000 == 0 && block != 0) {
            printf("已读取 %d MB...\r\n", (block * BLOCK_SIZE) / (1024 * 1024));
        }
    }
    
    printf("数据读取和验证完成，写入和读取的数据一致！\n");
    
    
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
    
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
    
  /* USER CODE END Callback 1 */
}

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
