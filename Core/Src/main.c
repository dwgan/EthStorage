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
#include "fatfs.h"
#include "lwip.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/init.h"
#include "lwip/netif.h"
#include "netif/ethernet.h"
#include "ethernetif.h"
#include "lwip/timeouts.h"

#include "lwip/tcp.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LAN8720_PHY_ADDRESS  0x00  // �������Ӳ�����������ȷ��PHY��ַ

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FIL MyFile;       /* �ļ����� */
char data[] = "Hello, this is a test data.\r\n";
extern struct netif gnetif;
extern ETH_HandleTypeDef heth;

// ����ȫ�� TCP ���ƿ����
static struct tcp_pcb *tcp_server_pcb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// ��������
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_server_error(void *arg, err_t err);
static void tcp_server_close(struct tcp_pcb *tpcb);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
void tcp_server_init(void)
{
    /* �����µ� TCP ���ƿ� */
    tcp_server_pcb = tcp_new();

    if (tcp_server_pcb != NULL)
    {
        err_t err;

        /* �󶨵�ָ���˿� */
        err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, 5000);
        if (err == ERR_OK)
        {
            /* �����ƿ�ת��Ϊ����״̬ */
            tcp_server_pcb = tcp_listen(tcp_server_pcb);

            /* ���ý������ӵĻص����� */
            tcp_accept(tcp_server_pcb, tcp_server_accept);

//            printf("TCP �������������������˿� 5000\r\n");
        }
        else
        {
            /* ��ʧ�ܣ��ͷſ��ƿ� */
            memp_free(MEMP_TCP_PCB, tcp_server_pcb);
//            printf("TCP ��������ʧ��\r\n");
        }
    }
    else
    {
//        printf("�޷������µ� TCP ���ƿ�\r\n");
    }
}

/* �����µĿͻ�������ʱ������ */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    /* ���ý������ݵĻص����� */
    tcp_recv(newpcb, tcp_server_recv);

    /* ���ô������� */
    tcp_err(newpcb, tcp_server_error);

//    printf("�ͻ���������\r\n");

    return ERR_OK;
}

/* �����յ�����ʱ������ */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        /* �����ѹرգ��ر� TCP ���� */
        tcp_server_close(tpcb);
//        printf("�ͻ����ѶϿ�����\r\n");
        return ERR_OK;
    }
    else
    {
        /* ������յ������� */
        tcp_recved(tpcb, p->tot_len);  // ���½��մ���

        /* ������ͨ�����ڷ��� */
        HAL_UART_Transmit(&huart1, p->payload, p->len, HAL_MAX_DELAY);

//        /* ��ѡ���ڴ��ڴ�ӡ���յ������� */
//        printf("���յ����ݣ�%.*s\r\n", p->len, (char *)p->payload);

        /* �ͷŽ��յ��� pbuf */
        pbuf_free(p);

        return ERR_OK;
    }
}

/* �����ӷ�������ʱ������ */
static void tcp_server_error(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(err);
    struct tcp_pcb *tpcb = (struct tcp_pcb *)arg;
    if (tpcb != NULL)
    {
        tcp_server_close(tpcb);
    }
}

/* �ر� TCP ���Ӳ��ͷ���Դ */
static void tcp_server_close(struct tcp_pcb *tpcb)
{
    tcp_arg(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);

    tcp_close(tpcb);
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
    MX_DMA_Init();
    MX_SDIO_SD_Init();
    MX_USART1_UART_Init();
    MX_FATFS_Init();
    MX_LWIP_Init();
    /* USER CODE BEGIN 2 */
    setvbuf(stdout, NULL, _IONBF, 0);
    
    /* �����ļ�ϵͳ */
    FRESULT res = f_mount(&SDFatFS, SDPath, 1);
    if (res != FR_OK)
    {
        printf("f_mount error: %d\n", res);
        if (res == FR_NO_FILESYSTEM)
        {
            printf("No filesystem, trying to format...\n");
            // ��ʽ�� SD ��
            res = f_mkfs(SDPath, FM_ANY, 0, NULL, 0);
            if (res == FR_OK)
            {
                printf("Format success, trying to mount again...\n");
                res = f_mount(&SDFatFS, SDPath, 1);
                if (res == FR_OK)
                {
                    printf("Mount success after format\n");
                }
                else
                {
                    printf("Mount failed after format, error: %d\n", res);
                    while(1);
                }
            }
            else
            {
                printf("Format failed, error: %d\n", res);
                while(1);
            }
        }
        else
        {
            while(1);
        }
    }
    else
    {
        printf("Mount success\n");
    }
    
    /* ���ļ�����д�� */
    res = f_open(&MyFile, "0:/data.txt", FA_OPEN_APPEND | FA_WRITE);
    if (res == FR_OK)
    {
        UINT byteswritten;
        
        /* �� data �����ظ�д���ļ� 100 �� */
        for (int i = 0; i < 1000; i++)
        {
            /* д������ */
            res = f_write(&MyFile, data, strlen(data), &byteswritten);
            if ((res == FR_OK) && (byteswritten == strlen(data)))
            {
                printf("Data written successfully (%d/100)\n", i + 1);
            }
            else
            {
                printf("Failed to write data at iteration %d, error: %d\n", i + 1, res);
                break;  // ���������˳�ѭ��
            }
        }
        
        /* �ر��ļ� */
        f_close(&MyFile);
    }
    else
    {
        printf("Failed to open file for writing, error: %d\n", res);
    }
    
    /* ��ȡ�ļ����� */
    res = f_open(&MyFile, "0:/data.txt", FA_READ);
    if (res == FR_OK)
    {
        char buffer[64];
        UINT bytesread;
        
        printf("File content:\n");
        do
        {
            res = f_read(&MyFile, buffer, sizeof(buffer)-1, &bytesread);
            if (res == FR_OK)
            {
                buffer[bytesread] = '\0';
                printf("%s", buffer);
            }
            else
            {
                printf("Failed to read file, error: %d\n", res);
                break;
            }
        } while (bytesread > 0);
        
        /* �ر��ļ� */
        f_close(&MyFile);
    }
    else
    {
        printf("Failed to open file for reading, error: %d\n", res);
    }
    /* ��ʼ�� TCP ������ */
    tcp_server_init();
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        MX_LWIP_Process();
        /* ���δʹ�� RTOS����Ҫ���ڵ��� */
        sys_check_timeouts();
        
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
