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
#define BLOCK_SIZE         512 // һ������ֽ���
#define DATA_SIZE_TO_WRITE 10*1024*1024 // 1MB = 1*1024*1024�ֽ�
#define DMA_NUM_BLOCKS_TO_WRITE 8 // ÿһ��DMAд��������
#define DMA_NUM_BLOCKS_TO_READ  1 // ÿһ��DMA�����������
#define NUM_TIMES_TO_WRITE DATA_SIZE_TO_WRITE/ (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_WRITE) // �ܹ�Ҫд��Ĵ���
#define NUM_TIMES_TO_READ  DATA_SIZE_TO_WRITE/ (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_READ) // �ܹ�Ҫд��Ĵ���

#define BUFFER_SIZE         16384         // ��������С

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t buffer1[BUFFER_SIZE];
//uint8_t buffer2[BUFFER_SIZE];
//uint8_t *active_buffer;  // ��ǰ��Ծ������
//uint8_t *processing_buffer;  // ���ڴ���Ļ�����

__ALIGN_BEGIN uint8_t buffer_TX[BUFFER_SIZE] __ALIGN_END;  // 4�ֽڶ��룬���ͻ�����
__ALIGN_BEGIN uint8_t buffer_RX[BLOCK_SIZE*DMA_NUM_BLOCKS_TO_READ] __ALIGN_END;  // 4�ֽڶ��룬���ջ�����

volatile uint8_t sd_write_complete = 0; // д����ɱ�־
volatile uint8_t sd_read_complete = 0;  // ��ȡ��ɱ�־

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

/* DMA�ص����� */
void HAL_SD_WriteCpltCallback(SD_HandleTypeDef *hsd)
{
    sd_write_complete = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
//    printf("SDIO �������\n");
    // ��Ӵ������߼��������Ի�λ SDIO
}

void generate_increasing_numbers(unsigned char *array, int num_bytes, int byte_size) {
    // ȷ��byte_size��1, 2, 3��4�ֽڵĵ�����
    if (byte_size < 1 || byte_size > 4) {
        printf("���������ֽ���������1, 2, 3��4\n");
        return;
    }

    // ȷ��num_bytes��byte_size�ı���
    if (num_bytes % byte_size != 0) {
        printf("������ֽ���������%d�ı���\n", byte_size);
        return;
    }

    int num_values = num_bytes / byte_size;  // ��Ҫ���ɵĵ�����������

    for (int i = 0; i < num_values; i++) {
        unsigned int value = i;  // ��������0��ʼ

        // ����byte_size��value��ÿ���ֽڰ��յ��ֽ���ǰ��˳���������
        for (int j = 0; j < byte_size; j++) {
            array[i * byte_size + j] = (value >> (8 * j)) & 0xFF;
        }
    }
}
int compare_buffers(const unsigned char *buffer1, const unsigned char *buffer2, int size) {
    for (int i = 0; i < size; i++) {
        if (*(buffer1+i) != *(buffer2+i)) {
            return i;  // ���ص�һ����ƥ����ֽ�����
        }
    }
    return -1;  // �����ȫ��ͬ������-1
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
//    // DMA ��ȡ���ʱ�л�������
//    uint8_t *temp = active_buffer;
//    active_buffer = processing_buffer;
//    processing_buffer = temp;
}

void process_data(uint8_t* buffer, uint32_t block) {
    // �����ﴦ�����ݣ����罫����д�������洢��������м���
    static int mismatch_index;
    mismatch_index = compare_buffers(buffer_TX+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ)%BUFFER_SIZE, buffer, BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ);
    if (mismatch_index != -1) {
        printf("������֤ʧ���ڿ� %d\r\n", block);
//        while (1); // ������ѭ�������ݴ���
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
    
    // ȷ�� SD ���ѳ�ʼ���ɹ�
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        printf("SD ��δ׼���ã�������ѭ����\n");
        while (1);
    }
    
    printf("SD ����ʼ���ɹ���\n");
    
    uint32_t BlockAdd = 0; // ��ʼ���
    
    /*--------------------------SD��д����----------------------------------*/
    
#define MAX_RETRIES 3          // ������Դ���
#define RETRY_DELAY_MS 1      // ����ǰ���ӳ٣����룩
    
    generate_increasing_numbers(buffer_TX, sizeof(buffer_TX), 4) ;
    
    // ��¼д�뿪ʼʱ��
    printf("��ʼд�����ݵ� SD ��...\n");
    uint32_t start_time = HAL_GetTick();

    for (uint32_t block = 0; block < NUM_TIMES_TO_WRITE; block++) {
        uint8_t retry = 0;
        HAL_StatusTypeDef status;
        
        // ����д�����
        do {
            // д��һ����
            status = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t*)buffer_TX+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_WRITE)%BUFFER_SIZE, BlockAdd, DMA_NUM_BLOCKS_TO_WRITE);
//            status = HAL_SD_WriteBlocks(&hsd, (uint8_t*)buffer_TX+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_WRITE)%BUFFER_SIZE, BlockAdd, DMA_NUM_BLOCKS_TO_WRITE,10);
            if (status == HAL_OK) {
                // �ȴ�д�����
                while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {}
                BlockAdd += DMA_NUM_BLOCKS_TO_WRITE; // ���¿����д����һ����
                break; // д��ɹ����˳�����ѭ��
            } else {
                printf("д��� %d ʧ�ܣ�״̬��%d������ %d/%d\r\n", 
                       BlockAdd, HAL_SD_GetError(&hsd), retry + 1, MAX_RETRIES);
                retry++;
                HAL_Delay(RETRY_DELAY_MS); // �ȴ�һ��ʱ��������
            }
        } while (retry < MAX_RETRIES);
        
        // ����ﵽ������Դ�����δ�ɹ�����¼���󲢼���
        if (status != HAL_OK) {
            printf("д��� %d ʧ�ܣ��ﵽ������Դ����������˿顣\n", BlockAdd);
            BlockAdd += DMA_NUM_BLOCKS_TO_WRITE; // ���¿�ţ���������ѭ��
            continue; // ����д����һ����
        }
        
        // ÿд��10000���ӡһ�ν���
        if ((block + 1) % 10000 == 0) {
            printf("��д�� %d MB...\r\n", ((block + 1) * (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_WRITE)) / (1024 * 1024));
        }

    }
    
    // ��¼д�����ʱ��
    uint32_t end_time = HAL_GetTick();
    
    // ������д��ʱ�䣨�룩
    float elapsed_time_sec = (end_time - start_time) / 1000.0f;
    
    // ����д���ٶȣ�MB/s��
    float write_speed = DATA_SIZE_TO_WRITE / (1024.0f * 1024.0f) / elapsed_time_sec;
    
    printf("����д����ɣ�\n");
    printf("��д��ʱ��: %.2f ��\n", elapsed_time_sec);
    printf("д���ٶ�: %.4f MB/s\n", write_speed);
    
    /*--------------------------SD����ȡ�ٶȲ��ԣ���ѡ��--------------------------*/
    // ��¼��ȡ��ʼʱ��
    printf("��ʼ��ȡ���ݴ� SD ��...\n");

    start_time = HAL_GetTick();
    for (uint32_t block = 0; block < NUM_TIMES_TO_READ; block++) 
    {
        // ���� DMA ��ȡ���ݵ���ǰ��Ծ������
        if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)buffer_RX, block*DMA_NUM_BLOCKS_TO_READ, DMA_NUM_BLOCKS_TO_READ) == HAL_OK) {
            // �ȴ���ȡ���
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER);
        } 
        else {
            printf("��ȡ�� %d ʧ�ܣ�״̬��%d\r\n", block, HAL_SD_GetError(&hsd));
//            while (1);  // ������ѭ������ȡ����
        }
        
        if (block % 10000 == 0 && block != 0) {
            printf("�Ѷ�ȡ %d MB...\r\n", (block * BLOCK_SIZE) / (1024 * 1024));
        }
    }
    // ��¼��ȡ����ʱ��
    end_time = HAL_GetTick();
    
    // �����ܶ�ȡʱ�䣨�룩
    elapsed_time_sec = (end_time - start_time) / 1000.0f;
    
    // �����ȡ�ٶȣ�MB/s��
    write_speed = DATA_SIZE_TO_WRITE / (1024.0f * 1024.0f) / elapsed_time_sec;
    
    printf("���ݶ�ȡ��ɣ�\n");
    printf("�ܶ�ȡʱ��: %.2f ��\n", elapsed_time_sec);
    printf("��ȡ�ٶ�: %.2f MB/s\n", write_speed);
    
    /*--------------------------SD�������ԣ���ѡ��--------------------------*/
    printf("��ʼ��ȡ���ݴ� SD ��...\n");
    
//    generate_increasing_numbers(buffer_TX, sizeof(buffer_TX), 2) ;
    
    for (uint32_t block = 0; block < NUM_TIMES_TO_READ; block++) {
        // ��ȡ���ݴ� SD ��
        if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)buffer_RX, block*DMA_NUM_BLOCKS_TO_READ, DMA_NUM_BLOCKS_TO_READ) == HAL_OK) {
            // �ȴ���ȡ���
//            printf("�ȴ���ȡ��� %d\r\n", block);

            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {}
            // ��֤����
//            printf("��֤���� %d\r\n", block);

            static int mismatch_index;
            mismatch_index = compare_buffers(buffer_TX+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ)%BUFFER_SIZE, buffer_RX, BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ);
            if (mismatch_index != -1) {
                printf("������֤ʧ���ڿ� %d\r\n", block);
                while (1); // ������ѭ�������ݴ���
            }
            else
            {   
//                printf("���ݶ�ȡ����֤��ɿ� %d\r\n", block);
            }
            
        } else {
            printf("��ȡ�� %d ʧ�ܣ�״̬��%d\r\n", block, HAL_SD_GetError(&hsd));
            while (1); // ������ѭ������ȡ����
        }
        
        if (block % 10000 == 0 && block != 0) {
            printf("�Ѷ�ȡ %d MB...\r\n", (block * BLOCK_SIZE) / (1024 * 1024));
        }
    }
    
    printf("���ݶ�ȡ����֤��ɣ�д��Ͷ�ȡ������һ�£�\n");
    
    
    
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
