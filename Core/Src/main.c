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
#define LAN8720_PHY_ADDRESS  0x00  // 根据你的硬件设计设置正确的PHY地址

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
FIL MyFile;       /* 文件对象 */
char data[] = "Hello, this is a test data.\r\n";
extern struct netif gnetif;
extern ETH_HandleTypeDef heth;

// 定义全局 TCP 控制块变量
static struct tcp_pcb *tcp_server_pcb;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// 函数声明
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
    /* 创建新的 TCP 控制块 */
    tcp_server_pcb = tcp_new();

    if (tcp_server_pcb != NULL)
    {
        err_t err;

        /* 绑定到指定端口 */
        err = tcp_bind(tcp_server_pcb, IP_ADDR_ANY, 5000);
        if (err == ERR_OK)
        {
            /* 将控制块转换为监听状态 */
            tcp_server_pcb = tcp_listen(tcp_server_pcb);

            /* 设置接受连接的回调函数 */
            tcp_accept(tcp_server_pcb, tcp_server_accept);

//            printf("TCP 服务器已启动，监听端口 5000\r\n");
        }
        else
        {
            /* 绑定失败，释放控制块 */
            memp_free(MEMP_TCP_PCB, tcp_server_pcb);
//            printf("TCP 服务器绑定失败\r\n");
        }
    }
    else
    {
//        printf("无法创建新的 TCP 控制块\r\n");
    }
}

/* 当有新的客户端连接时被调用 */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    /* 设置接收数据的回调函数 */
    tcp_recv(newpcb, tcp_server_recv);

    /* 设置错误处理函数 */
    tcp_err(newpcb, tcp_server_error);

//    printf("客户端已连接\r\n");

    return ERR_OK;
}

/* 当接收到数据时被调用 */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        /* 连接已关闭，关闭 TCP 连接 */
        tcp_server_close(tpcb);
//        printf("客户端已断开连接\r\n");
        return ERR_OK;
    }
    else
    {
        /* 处理接收到的数据 */
        tcp_recved(tpcb, p->tot_len);  // 更新接收窗口

        /* 将数据通过串口发送 */
        HAL_UART_Transmit(&huart1, p->payload, p->len, HAL_MAX_DELAY);

//        /* 可选：在串口打印接收到的数据 */
//        printf("接收到数据：%.*s\r\n", p->len, (char *)p->payload);

        /* 释放接收到的 pbuf */
        pbuf_free(p);

        return ERR_OK;
    }
}

/* 当连接发生错误时被调用 */
static void tcp_server_error(void *arg, err_t err)
{
    LWIP_UNUSED_ARG(err);
    struct tcp_pcb *tpcb = (struct tcp_pcb *)arg;
    if (tpcb != NULL)
    {
        tcp_server_close(tpcb);
    }
}

/* 关闭 TCP 连接并释放资源 */
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
    
    /* 挂载文件系统 */
    FRESULT res = f_mount(&SDFatFS, SDPath, 1);
    if (res != FR_OK)
    {
        printf("f_mount error: %d\n", res);
        if (res == FR_NO_FILESYSTEM)
        {
            printf("No filesystem, trying to format...\n");
            // 格式化 SD 卡
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
    
    /* 打开文件进行写入 */
    res = f_open(&MyFile, "0:/data.txt", FA_OPEN_APPEND | FA_WRITE);
    if (res == FR_OK)
    {
        UINT byteswritten;
        
        /* 将 data 数组重复写入文件 100 次 */
        for (int i = 0; i < 1000; i++)
        {
            /* 写入数据 */
            res = f_write(&MyFile, data, strlen(data), &byteswritten);
            if ((res == FR_OK) && (byteswritten == strlen(data)))
            {
                printf("Data written successfully (%d/100)\n", i + 1);
            }
            else
            {
                printf("Failed to write data at iteration %d, error: %d\n", i + 1, res);
                break;  // 发生错误，退出循环
            }
        }
        
        /* 关闭文件 */
        f_close(&MyFile);
    }
    else
    {
        printf("Failed to open file for writing, error: %d\n", res);
    }
    
    /* 读取文件内容 */
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
        
        /* 关闭文件 */
        f_close(&MyFile);
    }
    else
    {
        printf("Failed to open file for reading, error: %d\n", res);
    }
    /* 初始化 TCP 服务器 */
    tcp_server_init();
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        MX_LWIP_Process();
        /* 如果未使用 RTOS，需要定期调用 */
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
