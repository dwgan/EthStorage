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

#include "lwip/udp.h"
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
#define UDP_SEND_SIZE BLOCK_SIZE
#define CHECKSUM_LEN 2

#define BUFFER_DEPTH 2
#define TEMP_BUFF_SIZE 2048

#define LAN8720_PHY_ADDRESS  0x00  // �������Ӳ�����������ȷ��PHY��ַ


#define TARGET_IP "192.168.88.2"  // ��λ����IP��ַ
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
    INPUTING=1,
    WAIT4OUTPUT=2,
    OUTPUTING=3
}BUFF_STATE;

typedef struct
{
    uint8_t input_ptr;//��ǰ����д���buff
    uint8_t output_ptr;//��ǰ���������buff
    uint32_t index[BUFFER_DEPTH];//д��ָ�룬ָʾ��ǰbuff�ֽ�λ��
    BUFF_STATE buff_state[BUFFER_DEPTH];
    uint8_t buff[BUFFER_DEPTH][BUFFER_SIZE_W];
} UDP2SD_Buff_t;

typedef struct
{
    uint8_t input_ptr;//��ǰ����д���buff
    uint8_t output_ptr;//��ǰ���������buff
    uint32_t index[BUFFER_DEPTH];//д��ָ�룬ָʾ��ǰbuff�ֽ�λ��
    BUFF_STATE buff_state[BUFFER_DEPTH];
    uint8_t buff[BUFFER_DEPTH][BUFFER_SIZE_R];
} SD2UDP_Buff_t;

UDP2SD_Buff_t udp2sd_buff = {
    .input_ptr = 0,
    .output_ptr = 0,
    .index = {0},
    .buff_state = {WAIT4INPUT, WAIT4INPUT}, // ��ʼ�����л�����״̬
    .buff = {0}
};

SD2UDP_Buff_t sd2udp_buff = {
    .input_ptr = 0,
    .output_ptr = 0,
    .index = {0},
    .buff_state = {WAIT4INPUT, WAIT4INPUT}, // ��ʼ�����л�����״̬
    .buff = {0}
};

uint32_t last_tcp_receive_time = 0; // ��¼��һ�ν��յ�TCP���ݵ�ʱ��������룩

uint32_t temp_buff_len = 0;
//uint8_t temp_buff[TEMP_BUFF_SIZE];
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

uint32_t requested_block = 0;
volatile uint8_t request_received = 0;


typedef enum
{
    IDLE=0,
    SENDING_STREAM=1,
    SENDING_BLOCK=2
}UDP_SEND_STATE;
uint8_t udp_send_state = IDLE;

typedef enum
{
    REQUEST_STREAM=1,
    REQUEST_BLOCK=2,
    RESPONE_STREAM=3,
    RESPONE_BLOCK=4,
}REQUEST_CMD;
typedef struct
{
    REQUEST_CMD cmd;
    uint32_t start_addr;
    uint32_t end_addr;
} udp_head_t;
udp_head_t udp_reqeust={0};
udp_head_t udp_respone={0};
uint32_t socket_index=0;
uint8_t is_request_data = 0;
uint8_t is_request_block = 0;
uint8_t udp_block_send_done = 1;
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

/**************************������CRC���*********************************/
// ��4��CRC-16У����������ɶ���ʽ��(x^16 + x^12 + x^5 + 1)��
uint16_t crc16_table[] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

static inline uint16_t crc16_byte(uint16_t crc, const char data)
{
	return (crc << 8) ^ crc16_table[((crc >> 8) ^ data) & 0xff];
}

uint16_t CalCRC16(void *pData, int dwNumOfBytes)
{
    uint16_t	wCRC = 0;	// CRCУ����
    char *pbDataBuf = (char *)pData;
    while ( 0 != (dwNumOfBytes--) )
        wCRC = crc16_byte(wCRC, *pbDataBuf++);
    return	wCRC;
}
/**************************������CRC���*********************************/


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
    if (udp_send_state == SENDING_STREAM)
    {
        if (sd2udp_buff.buff_state[sd2udp_buff.input_ptr] == WAIT4INPUT && sdio_read_done == 1)
        {
            if (read_address < *write_address)
            {
                // ���ʣ�������㹻һ��buff��ֱ�Ӷ�һ��buff�����ʣ�����ݲ���һ��buff���ж��ٶ�����
                uint32_t read_num = *write_address - read_address;
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
                    sdio_read_done = 0;                
                    //                printf("��ȡ״̬��%d����%d��block\n", status, read_address);
                }
            }
        }
    }
    else if (udp_send_state == SENDING_BLOCK)
    {
        if (sd2udp_buff.buff_state[sd2udp_buff.input_ptr] == WAIT4INPUT && sdio_read_done == 1)
        {
            if (is_request_block == 1)
            {
                // ��һ�����������
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1)
                {
                    int status = HAL_SD_ReadBlocks_DMA(&hsd, sd2udp_buff.buff[sd2udp_buff.input_ptr], read_address, UDP_SEND_SIZE/BLOCK_SIZE);
                    sd2udp_buff.index[sd2udp_buff.input_ptr] = UDP_SEND_SIZE;
                    sd2udp_buff.buff_state[sd2udp_buff.input_ptr] = INPUTING;
                    sdio_read_done = 0;
                    is_request_block = 0;
                }
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
        printf("д��״̬��%d����%d��block����ǰindex��%d\n", 666, *write_address, udp2sd_buff.index[udp2sd_buff.output_ptr] );
        
    }
}


/**************************�����ǻ������***********************************/
/**************************��������̫�����***********************************/
//ip_addr_t client_addr;  // ����ͻ��˵�IP��ַ
//u16_t client_port;      // ����ͻ��˵Ķ˿ں�
void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        if (g_Config.workMode == WRITE) {
            // Existing code for WRITE mode
            
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
        }
        if (g_Config.workMode == READ) {
            // �����ȡ����
            if (p->len >= sizeof(uint32_t)) {
                
                memcpy(&udp_reqeust, p->payload, sizeof(udp_reqeust));
                
                udp_reqeust.start_addr = lwip_ntohl(udp_reqeust.start_addr);
                udp_reqeust.end_addr = lwip_ntohl(udp_reqeust.end_addr);
                // ���������ѽ��յı�־
                if (udp_reqeust.cmd == REQUEST_STREAM)
                {
                    read_address = udp_reqeust.start_addr;
                    *write_address = udp_reqeust.end_addr;
                    udp_send_state = SENDING_STREAM;
                    socket_index = 0;
                }
                else if (udp_reqeust.cmd == REQUEST_BLOCK)
                {
                    if (udp_block_send_done) // ����ȵ�����������ݰ����ͳ�ȥ���ٿ�ʼ������һ������
                    {
                        udp_block_send_done = 0;
                        udp_send_state = SENDING_BLOCK;
                        read_address = udp_reqeust.start_addr;
                        is_request_block = 1;
                    }
                }
//                request_received = 1;
//                // ����ͻ��˵ĵ�ַ�Ͷ˿ڣ��Ա㷢������
//                client_addr = *addr;
//                client_port = port;
            }
        }
        pbuf_free(p);
    }
}


void udp_send_task()
{
//    static uint32_t udp_index = 0;
    struct pbuf *udp_buf;
    uint32_t send_len=0;
    err_t status;
    static uint32_t send_index=0;
    static uint8_t first_pack = 1;
    if (udp_send_state == SENDING_STREAM)
    {
        udp_respone.cmd = RESPONE_STREAM;
        if (sd2udp_buff.buff_state[sd2udp_buff.output_ptr] == WAIT4OUTPUT)
        {
            while (send_index < sd2udp_buff.index[sd2udp_buff.output_ptr])
            {
                udp_respone.start_addr = lwip_ntohl(socket_index);
                send_len = sd2udp_buff.index[sd2udp_buff.output_ptr] - send_index;
                send_len = send_len > UDP_SEND_SIZE ? UDP_SEND_SIZE: send_len;
                // ��buff�е�����ͨ��UDP���ͻ��ض� IP �Ͷ˿�
                udp_buf = pbuf_alloc(PBUF_TRANSPORT, send_len+sizeof(udp_respone), PBUF_RAM);
                if (udp_buf != NULL) {
                    memcpy(udp_buf->payload, (void*)&udp_respone, sizeof(udp_respone));
                    memcpy((uint8_t *)udp_buf->payload+sizeof(udp_respone), sd2udp_buff.buff[sd2udp_buff.output_ptr]+send_index, send_len);
                    status = udp_send(udp_send_pcb, udp_buf);  // ��������
                    //                HAL_Delay(1);
                    socket_index++;
                    pbuf_free(udp_buf);  // �ͷ� pbuf
                }
//                if (udp_buf == NULL || status != ERR_OK)
//                {
//                    printf("udp error:%d\n", status);
//                }
//                            HAL_UART_Transmit(&huart1, (uint8_t *)&sd2udp_buff.buff[sd2udp_buff.output_ptr]+send_index, send_len, HAL_MAX_DELAY);
                //                        printf("UDP����״̬��%d�����͵�%d��block\n", status, send_len);
                //                HAL_Delay(2);
                send_index += send_len;
                if (first_pack && send_index >= UDP_SEND_SIZE)
                {
                    HAL_Delay(10);
                    first_pack = 0;
                    return;
                }
            }
            send_index = 0;
            sd2udp_buff.index[sd2udp_buff.output_ptr] = 0;
            sd2udp_buff.buff_state[sd2udp_buff.output_ptr] = WAIT4INPUT;
            sd2udp_buff.output_ptr = (sd2udp_buff.output_ptr + 1) % BUFFER_DEPTH;
        }

        if (read_address >= *write_address&& sd2udp_buff.buff_state[sd2udp_buff.output_ptr] == WAIT4INPUT)
        {
            udp_send_state = IDLE;
        }
    }
    else if (udp_send_state == SENDING_BLOCK)
    {
        udp_respone.cmd = RESPONE_BLOCK;
        if (sd2udp_buff.buff_state[sd2udp_buff.output_ptr] == WAIT4OUTPUT)
        {
            udp_respone.start_addr = lwip_ntohl(read_address);
            udp_respone.end_addr = lwip_ntohl(read_address+UDP_SEND_SIZE/BLOCK_SIZE-1);
            udp_buf = pbuf_alloc(PBUF_TRANSPORT, UDP_SEND_SIZE+sizeof(udp_respone)+CHECKSUM_LEN, PBUF_RAM);
            if (udp_buf != NULL) {
                memcpy(udp_buf->payload, (void*)&udp_respone, sizeof(udp_respone));
                memcpy((uint8_t *)udp_buf->payload+sizeof(udp_respone), sd2udp_buff.buff[sd2udp_buff.output_ptr], UDP_SEND_SIZE);
                uint16_t crc = CalCRC16(udp_buf->payload, UDP_SEND_SIZE+sizeof(udp_respone));
                memcpy((uint8_t *)udp_buf->payload+sizeof(udp_respone)+UDP_SEND_SIZE, (void *)&crc, CHECKSUM_LEN);
                status = udp_send(udp_send_pcb, udp_buf);  // ��������
                pbuf_free(udp_buf);  // �ͷ� pbuf
                HAL_UART_Transmit(&huart1, sd2udp_buff.buff[sd2udp_buff.output_ptr], UDP_SEND_SIZE+CHECKSUM_LEN, HAL_MAX_DELAY);
                udp_block_send_done = 1;
	            sd2udp_buff.index[sd2udp_buff.output_ptr] = 0;
	            sd2udp_buff.buff_state[sd2udp_buff.output_ptr] = WAIT4INPUT;
	            sd2udp_buff.output_ptr = (sd2udp_buff.output_ptr + 1) % BUFFER_DEPTH;
	            if (sd2udp_buff.buff_state[sd2udp_buff.output_ptr] == WAIT4INPUT)
	            {
	                udp_send_state = IDLE;
	            }
            }
        }
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
        if (g_Config.workMode == READ)
        {
            printf("��ǰ�Ƕ�ȡģʽ\n");
        }
        else if (g_Config.workMode == WRITE)
        {
            printf("��ǰ��д��ģʽ\n");
        }
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
        udp_receive_init();  // Add this line to initialize UDP receive in READ mode
        
        HAL_Delay(3000);
    }
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
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
        MX_LWIP_Process();
//        HAL_Delay(1);
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
