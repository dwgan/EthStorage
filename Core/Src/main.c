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

#include "lwip/tcp.h"
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
//#define BUFFER_SIZE_W         DMA_NUM_BLOCKS_TO_WRITE*BLOCK_SIZE         // ��������С
#define BUFFER_SIZE_W         5000// ��������С
#define BUFFER_SIZE_R         DMA_NUM_BLOCKS_TO_READ*BLOCK_SIZE         // ��������С
#define BUFFER_DEPTH 2
#define TEMP_BUFF_SIZE 2048

#define LAN8720_PHY_ADDRESS  0x00  // �������Ӳ�����������ȷ��PHY��ַ

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
    READY2WRITE=0,
//    WRITING,
    READY2READ=1,
    READING=2
}BUFF_STATE;
typedef struct
{
    uint8_t write_prt;//��ǰ����д���buff
    uint8_t write_prt_next;//��ǰ����д���buff
    uint8_t read_prt;//��ǰ����д���buff
    uint8_t read_prt_next;//��ǰ����д���buff
    uint32_t index[BUFFER_DEPTH];//д��ָ�룬ָʾ��ǰbuff�ֽ�λ��
    BUFF_STATE buff_state[BUFFER_DEPTH];
    uint8_t buff[BUFFER_DEPTH][BUFFER_SIZE_W];
} TCP2SD_Buff_t;

typedef struct
{
    uint8_t buff_r[2][BUFFER_SIZE_R];
} SD2TCP_Buff_t;

TCP2SD_Buff_t tcp2sd_buff = {
    .write_prt = 0,
    .write_prt_next = 1,
    .read_prt = 0,
    .read_prt_next = 1,
    .index = {0},
    .buff_state = {READY2WRITE, READY2WRITE}, // ��ʼ�����л�����״̬
    .buff = {0}
};
uint32_t last_tcp_receive_time = 0; // ��¼��һ�ν��յ�TCP���ݵ�ʱ��������룩


SD2TCP_Buff_t sd2tcp_buff={0};
uint32_t temp_buff_len = 0;
uint8_t temp_buff[TEMP_BUFF_SIZE];
struct tcp_pcb *global_pcb = NULL;        // ������ͣ�ͻָ� TCP ����

int write_enable = 1;
int read_enable = 1;
int read_done = 1;
uint32_t write_address = 0;
uint32_t read_address = 0;
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


void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

//    HAL_SD_GetCardState(hsd);
    write_enable = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
//    printf("SDIO �������\n");
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

int compare_buffers(unsigned char *buffer1, unsigned char *buffer2, int size) {
    for (int i = 0; i < size; i++) {
        if (*(buffer1+i) != *(buffer2+i)) {
            return i;  // ���ص�һ����ƥ����ֽ�����
        }
    }
    return -1;  // �����ȫ��ͬ������-1
}

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd)
{
//    HAL_SD_GetCardState(hsd);
    
//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    if (!read_enable)
    {
        read_enable = 1;
        read_done = 1;
    }
}

void process_data(uint8_t* buffer, uint32_t block) {
    // �����ﴦ�����ݣ����罫����д�������洢��������м���
    static int mismatch_index;
    mismatch_index = compare_buffers(tcp2sd_buff.buff[0]+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ)%BUFFER_SIZE_W, buffer, BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ);
    if (mismatch_index != -1) {
        printf("������֤ʧ���ڿ� %d\r\n", block);
//        while (1); // ������ѭ�������ݴ���
    }
}

void sdio_write_test()
{    
    // ��¼д�뿪ʼʱ��
    printf("��ʼд�����ݵ� SD ��...\n");
    uint32_t start_time = HAL_GetTick();
    while(1)
    {
        if (write_address < DATA_SIZE_TO_WRITE/BLOCK_SIZE)
        {
            if (write_enable)
            {
//                if (!(HAL_SD_GetCardState(&hsd) & HAL_SD_CARD_TRANSFER))
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1)
                {
                    int status = HAL_SD_WriteBlocks_DMA(&hsd, tcp2sd_buff.buff[0], write_address, DMA_NUM_BLOCKS_TO_WRITE);
                    if (status != HAL_OK) {
                        printf("�ж�д�����:%d\n", status);
                        continue;
                    }
                    write_address += DMA_NUM_BLOCKS_TO_WRITE;
                    write_enable = 0;
                    
                    if (write_address % (DMA_NUM_BLOCKS_TO_WRITE*2048) == 0) {
                        printf("��д�� %d MB...\r\n", (write_address * BLOCK_SIZE) / (1024 * 1024));
                    }
                }
            }
            
        }
        else
        {
            break;
        }
    }
    
    // ��¼д�����ʱ��
    uint32_t end_time = HAL_GetTick();
    
    // ������д��ʱ�䣨�룩
    float elapsed_time_sec = (end_time - start_time) / 1000.0f;
    
    // ����д���ٶȣ�MB/s��
    float write_speed = DATA_SIZE_TO_WRITE / (1024.0f * 1024.0f) / elapsed_time_sec;
    
    printf("����д����ɣ�\n");
    printf("��д��ʱ��: %.2f ��\n", (double)elapsed_time_sec);
    printf("д���ٶ�: %.4f MB/s\n", (double)write_speed);
}

void sdio_read_speed_test()
{
    
    /*--------------------------SD����ȡ�ٶȲ���--------------------------*/
    // ��¼��ȡ��ʼʱ��
    printf("��ʼ��ȡ���ݴ� SD ��...\n");

    uint32_t start_time = HAL_GetTick();
    while(1)
    {
        if (read_address < DATA_SIZE_TO_WRITE/BLOCK_SIZE)
        {
            if (read_enable)
            {
//                if (!(HAL_SD_GetCardState(&hsd) & HAL_SD_CARD_TRANSFER))
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1)
                {
                    int status = HAL_SD_ReadBlocks_DMA(&hsd, sd2tcp_buff.buff_r[0] + (read_address * BLOCK_SIZE)%sizeof(sd2tcp_buff.buff_r[0]), read_address, DMA_NUM_BLOCKS_TO_READ);
                    if (status != HAL_OK) {
                        printf("�ж϶�ȡ����:%d\nBlock: %d", status, read_address);
                        continue;
                    }
                    read_address += DMA_NUM_BLOCKS_TO_READ;
                    read_enable = 0;
                    
//                    if (write_address % (DMA_NUM_BLOCKS_TO_READ*2048) == 0) {
//                        printf("�Ѷ�ȡ %d MB...\r\n", (read_address * BLOCK_SIZE) / (1024 * 1024));
//                    }
                }
            }
            
        }
        else
        {
            break;
        }
    }
    // ��¼��ȡ����ʱ��
    uint32_t end_time = HAL_GetTick();
    
    // �����ܶ�ȡʱ�䣨�룩
    float elapsed_time_sec = (end_time - start_time) / 1000.0f;
    
    // �����ȡ�ٶȣ�MB/s��
    float write_speed = DATA_SIZE_TO_WRITE / (1024.0f * 1024.0f) / elapsed_time_sec;
    
    printf("���ݶ�ȡ��ɣ�\n");
    printf("�ܶ�ȡʱ��: %.2f ��\n", (double)elapsed_time_sec);
    printf("��ȡ�ٶ�: %.2f MB/s\n", (double)write_speed);
    
}

void sdio_read_compare()
{
    /*--------------------------SD��������--------------------------*/
    printf("��ʼ��ȡ���ݴ� SD ��...\n");
    HAL_Delay(100);
    memset(sd2tcp_buff.buff_r[0],0,sizeof(sd2tcp_buff.buff_r[0]));
    read_address = 0;
    read_enable = 1;
    read_done = 0;
    while(1)
    {
        if (read_address < DATA_SIZE_TO_WRITE/BLOCK_SIZE)
        {
            if (read_enable)
            {
                if (read_done)
                {
                    static int mismatch_index;
                    mismatch_index = compare_buffers(tcp2sd_buff.buff[0]+((read_address-DMA_NUM_BLOCKS_TO_READ) * BLOCK_SIZE)%BUFFER_SIZE_W, sd2tcp_buff.buff_r[0], BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ);
                    if (mismatch_index != -1) {
                        printf("������֤ʧ���ڿ� %d\r\n", read_address-DMA_NUM_BLOCKS_TO_READ);
                        while (1); // ������ѭ�������ݴ���
                    }
                    else
                    {
//                        printf("������֤�ɹ��ڿ� %d\r\n", read_address-DMA_NUM_BLOCKS_TO_READ);
                    }
                    read_done = 0;
                }
                //                if (!(HAL_SD_GetCardState(&hsd) & HAL_SD_CARD_TRANSFER))
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1)
                {
                    int status = HAL_SD_ReadBlocks_DMA(&hsd, sd2tcp_buff.buff_r[0], read_address, DMA_NUM_BLOCKS_TO_READ);
                    if (status != HAL_OK) {
                        printf("�ж϶�ȡ����:%d\nBlock: %d", status, read_address);
                        continue;
                    }
                    read_address += DMA_NUM_BLOCKS_TO_READ;
                    read_enable = 0;
                }
            }
            
        }
        else
        {
            break;
        }
    }
    printf("���ݶ�ȡ����֤��ɣ�д��Ͷ�ȡ������һ�£�\n");
}
/**************************������SD�����*********************************/

/**************************�����ǻ������***********************************/
void uart_send_task(void) {
    // ��鵱ǰ�������Ƿ�ɶ�����������
    if (tcp2sd_buff.buff_state[tcp2sd_buff.read_prt] == READY2READ) {
        // ͨ�� UART ��������
        HAL_UART_Transmit(&huart1, tcp2sd_buff.buff[tcp2sd_buff.read_prt], tcp2sd_buff.index[tcp2sd_buff.read_prt], HAL_MAX_DELAY);
        
        // ��ǻ�����Ϊ��д
        tcp2sd_buff.buff_state[tcp2sd_buff.read_prt] = READY2WRITE;
        tcp2sd_buff.index[tcp2sd_buff.read_prt] = 0;

        // �л�����һ��������
        tcp2sd_buff.read_prt = (tcp2sd_buff.read_prt + 1) % BUFFER_DEPTH;
    }
//    else if (tcp2sd_buff.buff_state[(tcp2sd_buff.read_prt + 1) % BUFFER_DEPTH] == READY2READ)
//    {
//        tcp2sd_buff.read_prt = (tcp2sd_buff.read_prt + 1) % BUFFER_DEPTH;
//    }
}
#define TIMEOUT_THRESHOLD 1000  // ��ʱ��ֵ (ms) ������Ҫ����
uint8_t is_udp_rcv=0;
void check_timeout_and_flush(void) {
    uint32_t current_time = HAL_GetTick();
    
    // ����Ƿ�ʱ
    if ((current_time - last_tcp_receive_time) > TIMEOUT_THRESHOLD && is_udp_rcv) {
        is_udp_rcv = 0;
        // �����ǰ��������������δ���ͣ���ǿ�Ʒ���
        if (tcp2sd_buff.index[tcp2sd_buff.write_prt] > 0) {
            tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] = READY2READ;
            tcp2sd_buff.write_prt = (tcp2sd_buff.write_prt + 1) % BUFFER_DEPTH;
            uart_send_task();  // ����UART����
        }
    }
}


/**************************�����ǻ������***********************************/
/**************************��������̫�����***********************************/
#include "lwip/udp.h"
#define TARGET_IP "192.168.88.2"  // ��λ����IP��ַ
#define TARGET_IP1 "192.168.88.3"  // ��λ����IP��ַ
#define TARGET_PORT 5006          // ��λ�����յĶ˿ں�

// ȫ�ֱ������ڴ洢 UDP ����
struct udp_pcb *udp_pcb;
struct udp_pcb *udp_send_pcb;
struct udp_pcb *udp_send_pcb1;

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
            if (tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] == READY2WRITE) {
                uint32_t available_space = BUFFER_SIZE_W - tcp2sd_buff.index[tcp2sd_buff.write_prt];  // ��ǰ������ʣ��ռ�
                
                if (available_space >= remaining_data) {
                    // ���������㹻�Ŀռ�д���������ݰ�
                    memcpy(tcp2sd_buff.buff[tcp2sd_buff.write_prt] + tcp2sd_buff.index[tcp2sd_buff.write_prt], data_ptr, remaining_data);
                    tcp2sd_buff.index[tcp2sd_buff.write_prt] += remaining_data;  // ���»�����д��λ��
                    remaining_data = 0;  // �����Ѿ�ȫ��д��
                } else {
                    // ������û���㹻�ռ䣬��Ҫ������ݰ�
                    memcpy(tcp2sd_buff.buff[tcp2sd_buff.write_prt] + tcp2sd_buff.index[tcp2sd_buff.write_prt], data_ptr, available_space);
                    tcp2sd_buff.index[tcp2sd_buff.write_prt] += available_space;
                    remaining_data -= available_space;  // �����Ѵ����������
                    data_ptr += available_space;  // �ƶ�����ָ��
                    
                    // ��ǵ�ǰ������Ϊ�ɶ�
                    tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] = READY2READ;
                    
                    // �л�����һ��������
                    tcp2sd_buff.write_prt = (tcp2sd_buff.write_prt + 1) % BUFFER_DEPTH;
                    
                    // ����µĻ������Ƿ��д
                    if (tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] != READY2WRITE) {
                        // �����һ��������Ҳ����д��������Ҫ��������ȴ�����
                        // ������Լ��϶������ݻ������ȴ������߼�
                        break;
                    }
                    
                    // �����»�������д������
                    tcp2sd_buff.index[tcp2sd_buff.write_prt] = 0;
                }
            }
        }
        
        // ͨ�� UDP �����յ������ݷ��ͻ��ض� IP �Ͷ˿�
        struct pbuf *udp_buf = pbuf_alloc(PBUF_TRANSPORT, remaining_data, PBUF_RAM);
        if (udp_buf != NULL) {
            memcpy(udp_buf->payload, data_ptr, p->len);
            udp_send(udp_send_pcb, udp_buf);  // ��������
            pbuf_free(udp_buf);  // �ͷ� pbuf
        }
        
        // ͨ�� UDP �����յ������ݷ��ͻ��ض� IP �Ͷ˿�
        struct pbuf *udp_buf1 = pbuf_alloc(PBUF_TRANSPORT, remaining_data, PBUF_RAM);
        if (udp_buf1 != NULL) {
            memcpy(udp_buf1->payload, data_ptr, p->len);
            udp_send(udp_send_pcb1, udp_buf1);  // ��������
            pbuf_free(udp_buf1);  // �ͷ� pbuf
        }
        
        // �ͷ� pbuf �ṹ
        pbuf_free(p);
    }
}


void my_udp_init(void) {
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

// ��ʼ������Ŀ��
void udp_send_init1(void) {
    ip_addr_t dest_ip_addr;
    ipaddr_aton(TARGET_IP1, &dest_ip_addr);  // ת��Ŀ��IPΪ��ȷ�ĸ�ʽ

    udp_send_pcb1 = udp_new();
    if (udp_send_pcb1 != NULL) {
        udp_connect(udp_send_pcb1, &dest_ip_addr, TARGET_PORT);  // ���ӵ�Ŀ��IP�Ͷ˿�
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
    
    // ȷ�� SD ���ѳ�ʼ���ɹ�
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        printf("SD ��δ׼���ã�������ѭ����\n");
    }
    else
    {
        printf("SD ����ʼ���ɹ���\n");
    }
    my_udp_init();
    udp_send_init();
    udp_send_init1();
    /* USER CODE END 2 */
    
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        MX_LWIP_Process();
        uart_send_task();   // ���� UART ����
        check_timeout_and_flush();
        
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
