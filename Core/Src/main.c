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
#include "lwip.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lwip/tcp.h"
#include "main.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"
//#include "dma.h"

#include <stdio.h>
#include <string.h>
#include "lwip/init.h"
#include "lwip/netif.h"
#include "netif/ethernet.h"
#include "ethernetif.h"
#include "lwip/timeouts.h"
#include <stddef.h>  // ���� offsetof ��

#include "globals.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define DATA_SIZE 524288000 //500*1024*1024
#define SD_CAPACITY 100 // ��λ MBytes
#define BLOCK_SIZE         512 // һ������ֽ���
#define DATA_SIZE_TO_WRITE 100*1024*1024 // 1MB = 1*1024*1024�ֽ�
#define DMA_NUM_BLOCKS_TO_WRITE 64 // ÿһ��DMAд��������
#define DMA_NUM_BLOCKS_TO_READ  16 // ÿһ��DMA�����������
#define NUM_TIMES_TO_WRITE DATA_SIZE_TO_WRITE/ (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_WRITE) // �ܹ�Ҫд��Ĵ���
#define NUM_TIMES_TO_READ  DATA_SIZE_TO_WRITE/ (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_READ) // �ܹ�Ҫд��Ĵ���
#define SD_CARD_CAPACITY_IN_BLOCKS SD_CAPACITY*1024*1024/BLOCK_SIZE
#define BUFFER_SIZE_W         DMA_NUM_BLOCKS_TO_WRITE*BLOCK_SIZE         // ��������С
//#define BUFFER_SIZE_W         2000// ��������С
#define BUFFER_SIZE_R         DMA_NUM_BLOCKS_TO_READ*BLOCK_SIZE         // ��������С
#define UDP_SEND_SIZE 1024

#define BUFFER_DEPTH 2
#define TEMP_BUFF_SIZE 2048

#define LAN8720_PHY_ADDRESS  0x00  // �������Ӳ�����������ȷ��PHY��ַ


#include "lwip/udp.h"
#define TARGET_IP "192.168.88.78"  // ��λ����IP��ַ
#define TARGET_PORT 5007          // ��λ�����յĶ˿ں�

#define TIMEOUT_THRESHOLD 1000  // ��ʱ��ֵ (ms) ������Ҫ����
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



#pragma pack(1)
// buff_state
// 0: ready for writing data to buff
// 1: writing data to buff in processing
// 2: ready for reading data from buff
// 3: reading data from buff in processing
typedef enum
{
    WAIT4INPUT=0,
    INPUTING,
    WAIT4OUTPUT=1,
    OUTPUTING=2
}BUFF_STATE;

typedef struct
{
    uint8_t input_ptr;//��ǰ����д���buff
//    uint8_t write_prt_next;//��ǰ����д���buff
    uint8_t output_ptr;//��ǰ����д���buff
//    uint8_t read_prt_next;//��ǰ����д���buff
    uint32_t index[BUFFER_DEPTH];//д��ָ�룬ָʾ��ǰbuff�ֽ�λ��
    BUFF_STATE buff_state[BUFFER_DEPTH];
    uint8_t buff[BUFFER_DEPTH][BUFFER_SIZE_W];
} UDP2SD_Buff_t;

typedef struct
{
    uint8_t input_ptr;//��ǰ����д���buff
//    uint8_t write_prt_next;//��ǰ����д���buff
    uint8_t output_ptr;//��ǰ����д���buff
//    uint8_t read_prt_next;//��ǰ����д���buff
    uint32_t index[BUFFER_DEPTH];//д��ָ�룬ָʾ��ǰbuff�ֽ�λ��
    BUFF_STATE buff_state[BUFFER_DEPTH];
    uint8_t buff[BUFFER_DEPTH][BUFFER_SIZE_R];
} SD2UDP_Buff_t;

UDP2SD_Buff_t udp2sd_buff = {
    .input_ptr = 0,
//    .write_prt_next = 1,
    .output_ptr = 0,
//    .read_prt_next = 1,
    .index = {0},
    .buff_state = {WAIT4INPUT, WAIT4INPUT}, // ��ʼ�����л�����״̬
    .buff = {0}
};

SD2UDP_Buff_t sd2udp_buff = {
    .input_ptr = 0,
//    .write_prt_next = 1,
    .output_ptr = 0,
//    .read_prt_next = 1,
    .index = {0},
    .buff_state = {WAIT4INPUT, WAIT4INPUT}, // ��ʼ�����л�����״̬
    .buff = {0}
};

uint32_t last_tcp_receive_time = 0; // ��¼��һ�ν��յ�TCP���ݵ�ʱ��������룩

uint32_t temp_buff_len = 0;
uint8_t temp_buff[TEMP_BUFF_SIZE];
struct tcp_pcb *global_pcb = NULL;        // ������ͣ�ͻָ� TCP ����

int sdio_write_done = 1;
int read_enable = 1;
int sdio_read_done = 1;
uint32_t *write_address = NULL;
uint32_t read_address = NULL;
uint8_t is_udp_rcv=0;


// ȫ�ֱ������ڴ洢 UDP ����
struct udp_pcb *udp_pcb;
struct udp_pcb *udp_send_pcb;
#pragma pack()
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//static err_t tcp_server_send_data(struct tcp_pcb *tpcb);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**************************������SD�����*********************************/
void sdio_init()
{
    write_address = &g_Config.sdioAddrW;
    read_address = g_Config.sdioAddrR;
}


void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
    if (!sdio_write_done && udp2sd_buff.buff_state[udp2sd_buff.output_ptr] == OUTPUTING)
    {
        sdio_write_done = 1;
        // ��ǻ�����Ϊ��д
        udp2sd_buff.buff_state[udp2sd_buff.output_ptr] = WAIT4INPUT;
        udp2sd_buff.index[udp2sd_buff.output_ptr] = 0;
        
        // �л�����һ��������
        udp2sd_buff.output_ptr = (udp2sd_buff.output_ptr + 1) % BUFFER_DEPTH;
    }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
//    printf("SDIO �������\n");
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
    if (!sdio_read_done && sd2udp_buff.buff_state[sd2udp_buff.input_ptr] == INPUTING)
    {
        sdio_read_done = 1;
        // ��ָ���
//        read_address += sd2udp_buff.index[sd2udp_buff.input_ptr];
        // buff�ɶ�
        sd2udp_buff.buff_state[sd2udp_buff.input_ptr] = WAIT4OUTPUT;
        // �л�����һ��buff
        sd2udp_buff.input_ptr = (sd2udp_buff.input_ptr + 1) % BUFFER_DEPTH;
    }
}

/**************************������SD�����*********************************/

/**************************�����ǻ������***********************************/
void sdio_write_task(void) {
    // ��鵱ǰ�������Ƿ�ɶ�����������
    if (udp2sd_buff.buff_state[udp2sd_buff.output_ptr] == WAIT4OUTPUT) {
        static uint32_t i = 0;
        // д��SD��
        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1) // ������ֻ�е�SDIO���߿��е�ʱ����ܹ�����д�룬�������
        {
            int status = HAL_SD_WriteBlocks_DMA(&hsd, udp2sd_buff.buff[udp2sd_buff.output_ptr], *write_address, DMA_NUM_BLOCKS_TO_WRITE);
            if (status != HAL_OK || i++ % 1000 == 0)
            {
                printf("д��״̬��%d����%d��block����ǰindex��%d\n", status, *write_address, udp2sd_buff.index[udp2sd_buff.output_ptr] );
            }
            *write_address += DMA_NUM_BLOCKS_TO_WRITE;
            udp2sd_buff.buff_state[udp2sd_buff.output_ptr] = OUTPUTING;
            sdio_write_done = 0;                
        }
    }
}

void sdio_read_task(void)
{
    uint32_t read_num=0;
    if (sd2udp_buff.buff_state[sd2udp_buff.input_ptr] == WAIT4INPUT)
    {
        if (read_address < *write_address)
        {
            // ���ʣ�������㹻һ��buff��ֱ�Ӷ�һ��buff�����ʣ�����ݲ���һ��buff���ж��ٶ�����
            read_num = *write_address - read_address;
            read_num =  read_num> DMA_NUM_BLOCKS_TO_READ? DMA_NUM_BLOCKS_TO_READ : read_num;
            // ��һ�����������
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1)
            {
                int status = HAL_SD_ReadBlocks_DMA(&hsd, sd2udp_buff.buff[sd2udp_buff.input_ptr], read_address, DMA_NUM_BLOCKS_TO_READ);
                read_address += read_num;
                // buff ind��
                sd2udp_buff.index[sd2udp_buff.input_ptr] = read_num * BLOCK_SIZE;
                // buff ״̬��
                sd2udp_buff.buff_state[sd2udp_buff.input_ptr] = INPUTING;
                sd2udp_buff.input_ptr = (sd2udp_buff.input_ptr + 1) % BUFFER_DEPTH;
                sdio_read_done = 0;                
//                printf("��ȡ״̬��%d����%d��block\n", status, read_address);
            }
        }
    }
}

void check_timeout_and_flush(void) {
    uint32_t current_time = HAL_GetTick();
    
    // ����Ƿ�ʱ
    if ((current_time - last_tcp_receive_time) > TIMEOUT_THRESHOLD && is_udp_rcv) {
        is_udp_rcv = 0;
        // �����ǰ��������������δ���ͣ���ǿ�Ʒ���
        if (udp2sd_buff.index[udp2sd_buff.input_ptr] > 0) {
            udp2sd_buff.buff_state[udp2sd_buff.input_ptr] = WAIT4OUTPUT;
            udp2sd_buff.input_ptr = (udp2sd_buff.input_ptr + 1) % BUFFER_DEPTH;
            sdio_write_task();  // ����UART����
        }
        write_config_to_flash(&g_Config);
//        write_single_variable_to_flash(FLASH_USER_START_ADDR + offsetof(GlobalConfig_t, sdioAddrW), &g_Config.sdioAddrW);
        printf("д��״̬��%d����%d��block����ǰindex��%d\n", 666, *write_address, udp2sd_buff.index[udp2sd_buff.output_ptr] );
        
    }
}


/**************************�����ǻ������***********************************/
/**************************��������̫�����***********************************/

void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        uint16_t remaining_data = p->len;  // ʣ�����ݵĳ���
        uint8_t *data_ptr = (uint8_t *)p->payload;  // ָ����յ���UDP����
        
        // �����������ʱ��
        last_tcp_receive_time = HAL_GetTick();
        is_udp_rcv = 1;
        // ����ǰ������
        while (remaining_data > 0) {
            // ��鵱ǰ�������Ƿ��д
            if (udp2sd_buff.buff_state[udp2sd_buff.input_ptr] == WAIT4INPUT) {
                uint32_t available_space = BUFFER_SIZE_W - udp2sd_buff.index[udp2sd_buff.input_ptr];  // ��ǰ������ʣ��ռ�
                
                if (available_space >= remaining_data) {
                    // ���������㹻�Ŀռ�д���������ݰ�
                    memcpy(udp2sd_buff.buff[udp2sd_buff.input_ptr] + udp2sd_buff.index[udp2sd_buff.input_ptr], data_ptr, remaining_data);
                    udp2sd_buff.index[udp2sd_buff.input_ptr] += remaining_data;  // ���»�����д��λ��
                    remaining_data = 0;  // �����Ѿ�ȫ��д��
                } else {
                    // ������û���㹻�ռ䣬��Ҫ������ݰ�
                    memcpy(udp2sd_buff.buff[udp2sd_buff.input_ptr] + udp2sd_buff.index[udp2sd_buff.input_ptr], data_ptr, available_space);
                    udp2sd_buff.index[udp2sd_buff.input_ptr] += available_space;
                    remaining_data -= available_space;  // �����Ѵ����������
                    data_ptr += available_space;  // �ƶ�����ָ��
                    
                    // ��ǵ�ǰ������Ϊ�ɶ�
                    udp2sd_buff.buff_state[udp2sd_buff.input_ptr] = WAIT4OUTPUT;
                    
                    // �л�����һ��������
                    udp2sd_buff.input_ptr = (udp2sd_buff.input_ptr + 1) % BUFFER_DEPTH;
                    
                    // ����µĻ������Ƿ��д
                    if (udp2sd_buff.buff_state[udp2sd_buff.input_ptr] != WAIT4INPUT) {
                        // �����һ��������Ҳ����д��������Ҫ��������ȴ�����
                        // ������Լ��϶������ݻ������ȴ������߼�
                        break;
                    }
                    
                    // �����»�������д������
                    udp2sd_buff.index[udp2sd_buff.input_ptr] = 0;
                }
            }
        }
        
        
        // �ͷ� pbuf �ṹ
        pbuf_free(p);
    }
}

void udp_send_task()
{
    uint32_t send_num=0;
    uint32_t send_index=0;
    err_t status;
    struct pbuf *udp_buf;
    if (sd2udp_buff.buff_state[sd2udp_buff.output_ptr] == WAIT4OUTPUT)
    {
        while (send_index < sd2udp_buff.index[sd2udp_buff.output_ptr])
        {
            send_num = sd2udp_buff.index[sd2udp_buff.output_ptr] - send_index;
            send_num = send_num > UDP_SEND_SIZE ? UDP_SEND_SIZE: send_num;
//            // ��buff�е�����ͨ��UDP���ͻ��ض� IP �Ͷ˿�
//            udp_buf = pbuf_alloc(PBUF_TRANSPORT, send_num, PBUF_RAM);
//            if (udp_buf != NULL) {
//                memcpy(udp_buf->payload, sd2udp_buff.buff[sd2udp_buff.output_ptr]+send_index, send_num);
//                status = udp_send(udp_send_pcb, udp_buf);  // ��������
//                pbuf_free(udp_buf);  // �ͷ� pbuf
//            }
            HAL_UART_Transmit(&huart1, (uint8_t *)&sd2udp_buff.buff[sd2udp_buff.output_ptr]+send_index, send_num, HAL_MAX_DELAY);
            
            //            printf("UDP����״̬��%d�����͵�%d��block\n", status, send_num);
            send_index += send_num;
        }
        sd2udp_buff.index[sd2udp_buff.output_ptr] = 0;
        sd2udp_buff.buff_state[sd2udp_buff.output_ptr] = WAIT4INPUT;
        sd2udp_buff.output_ptr = (sd2udp_buff.output_ptr + 1) % BUFFER_DEPTH;
    }
}

void udp_receive_init(void) {
    // ��� UDP ��ʼ������
    udp_pcb = udp_new();
    if (udp_pcb != NULL) {
        udp_bind(udp_pcb, IP_ADDR_ANY, 5005);
        udp_recv(udp_pcb, udp_receive_callback, NULL);
    }
}

// ��ʼ������Ŀ��
void udp_send_init(void) {
    ip_addr_t dest_ip_addr;
    ipaddr_aton(TARGET_IP, &dest_ip_addr);  // ת��Ŀ��IPΪ��ȷ�ĸ�ʽ

    udp_send_pcb = udp_new();
    if (udp_send_pcb != NULL) {
        udp_connect(udp_send_pcb, &dest_ip_addr, TARGET_PORT);  // ���ӵ�Ŀ��IP�Ͷ˿�
    }
}

/**************************��������̫�����*********************************/

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
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
    HAL_Delay(100);
    
    
    setvbuf(stdout, NULL, _IONBF, 0);
    printf("\n\nStart testing \n");
    
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
    
    GlobalConfig_t *g_Config_temp = (GlobalConfig_t *)malloc(sizeof(GlobalConfig_t));
    read_config_from_flash(g_Config_temp);
    // ���flash��û������
    if (g_Config_temp->verNum == 0 || g_Config_temp->verNum == 0xffffffff)
    {
        write_config_to_flash(&g_Config);
    }
    else
    {
        memcpy(&g_Config, g_Config_temp, sizeof(GlobalConfig_t));
    }
    free(g_Config_temp);
    
    // ȷ�� SD ���ѳ�ʼ���ɹ�
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        printf("SD ��δ׼���ã�������ѭ����\n");
    }
    else
    {
        sdio_init();
        printf("SD ����ʼ���ɹ���\n");
        printf("��ǰд��ָ�룺%d\n", *write_address);
        printf("��ǰ����ָ�룺%d\n", read_address);
    }
    
    if (g_Config.workMode == WRITE)
    {
        udp_receive_init();
    }
    if (g_Config.workMode == READ)
    {
        udp_send_init();
        HAL_Delay(5000);
    }
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        MX_LWIP_Process();
        
        if (g_Config.workMode == WRITE)
        {
            sdio_write_task();
            check_timeout_and_flush();
        }
        if (g_Config.workMode == READ)
        {
            sdio_read_task();
            udp_send_task();
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
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
