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
#define BUFFER_SIZE         DMA_NUM_BLOCKS_TO_WRITE*BLOCK_SIZE         // ��������С

#define LAN8720_PHY_ADDRESS  0x00  // �������Ӳ�����������ȷ��PHY��ַ

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//uint8_t buffer1[BUFFER_SIZE];
//uint8_t buffer2[BUFFER_SIZE];
uint8_t buffer1[512];
uint8_t buffer2[512];

volatile uint8_t *current_buffer = buffer1;    // ��ǰ���ڽ��յĻ�����
volatile uint8_t *write_buffer = NULL;         // ��ǰ����д��SD���Ļ�����

volatile uint32_t buffer_index = 0;            // ��ǰ��������д������
volatile uint8_t buffer_full = 0;              // ��־��ǰ�������Ƿ�����

volatile uint8_t sd_write_in_progress = 0;     // ��־�Ƿ����ڽ���SD��д��


__ALIGN_BEGIN uint8_t buffer_TX[BUFFER_SIZE] __ALIGN_END;  // 4�ֽڶ��룬���ͻ�����
__ALIGN_BEGIN uint8_t buffer_RX[BLOCK_SIZE*DMA_NUM_BLOCKS_TO_READ] __ALIGN_END;  // 4�ֽڶ��룬���ջ�����

volatile uint8_t sd_write_complete = 0; // д����ɱ�־
volatile uint8_t sd_read_complete = 0;  // ��ȡ��ɱ�־

extern struct netif gnetif;
extern ETH_HandleTypeDef heth;

int write_enable = 1;
int read_enable = 1;
int read_done = 1;
// ����ȫ�� TCP ���ƿ����
static struct tcp_pcb *tcp_server_pcb;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_server_error(void *arg, err_t err);
static void tcp_server_close(struct tcp_pcb *tpcb);
static err_t tcp_server_send_data(struct tcp_pcb *tpcb);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**************************������SD�����*********************************/
///* DMA�ص����� */
//void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
//{
//    sd_write_complete = 1;
////    sd_write_in_progress = 0;
//}
// д����ʼ��ַ���� block Ϊ��λ��
uint32_t write_address = 0;
uint32_t read_address = 0;

// ��ʼд������
void Start_Write(void) {
    if (HAL_SD_WriteBlocks_DMA(&hsd, buffer_TX, write_address, DMA_NUM_BLOCKS_TO_WRITE) != HAL_OK) {
        // д�������
        printf("�����ж�д�����\n");
    }
    HAL_SD_GetCardState(&hsd);
}
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    // ����д���ַ
//    write_address += DMA_NUM_BLOCKS_TO_WRITE;

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

    // ����Ƿ��и���������Ҫд��
    static uint32_t num=0;
//    if (num++ < DATA_SIZE_TO_WRITE/ DMA_NUM_BLOCKS_TO_WRITE/BLOCK_SIZE) {
//    }
//    printf("0x%4.x\n", HAL_SD_GetCardState(hsd));
    HAL_SD_GetCardState(hsd);
    write_enable = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    printf("SDIO �������\n");
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
int compare_buffers(unsigned char *buffer1, unsigned char *buffer2, int size) {
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
    HAL_SD_GetCardState(hsd);
    
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    if (!read_enable)
    {
        read_enable = 1;
        read_done = 1;
    }
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
                    int status = HAL_SD_WriteBlocks_DMA(&hsd, buffer_TX, write_address, DMA_NUM_BLOCKS_TO_WRITE);
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
    printf("��д��ʱ��: %.2f ��\n", elapsed_time_sec);
    printf("д���ٶ�: %.4f MB/s\n", write_speed);
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
                    int status = HAL_SD_ReadBlocks_DMA(&hsd, buffer_RX + (read_address * BLOCK_SIZE)%sizeof(buffer_RX), read_address, DMA_NUM_BLOCKS_TO_READ);
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
    printf("�ܶ�ȡʱ��: %.2f ��\n", elapsed_time_sec);
    printf("��ȡ�ٶ�: %.2f MB/s\n", write_speed);
    
}

void sdio_read_compare()
{
    /*--------------------------SD��������--------------------------*/
    printf("��ʼ��ȡ���ݴ� SD ��...\n");
    HAL_Delay(100);
    memset(buffer_RX,0,sizeof(buffer_RX));
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
                    mismatch_index = compare_buffers(buffer_TX+((read_address-DMA_NUM_BLOCKS_TO_READ) * BLOCK_SIZE)%BUFFER_SIZE, buffer_RX, BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ);
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
                    int status = HAL_SD_ReadBlocks_DMA(&hsd, buffer_RX, read_address, DMA_NUM_BLOCKS_TO_READ);
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

//void write_data_to_sd_card()
//{
//    static uint32_t BlockAdd = 1000;  // SD������ʼ���ַ
//    HAL_StatusTypeDef status;
//
//    if (sd_write_in_progress && write_buffer != NULL)
//    {
//        uint32_t data_length = BUFFER_SIZE;  // ���軺��������
//        uint8_t retry = 0;
//
//        // ������Ҫд��Ŀ���
//        uint32_t blocks_to_write = data_length / BLOCK_SIZE;
//
//        // д��SD��
//        do
//        {
//            status = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)write_buffer, BlockAdd, blocks_to_write);
//            if (status == HAL_OK)
//            {
//                // �ȴ�д�����
//                while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {}
//                BlockAdd += blocks_to_write;  // ���¿��ַ
//                break;
//            }
//            else
//            {
//                printf("д��� %lu ʧ�ܣ�״̬��%d������ %d/%d\r\n",
//                       BlockAdd, HAL_SD_GetError(&hsd), retry + 1, MAX_RETRIES);
//                retry++;
//                HAL_Delay(RETRY_DELAY_MS);
//            }
//        } while (retry < 1);
//
//        if (status != HAL_OK)
//        {
//            printf("д��� %lu ʧ�ܣ��ﵽ������Դ����������˿顣\n", BlockAdd);
//            BlockAdd += blocks_to_write;  // ����������Ŀ�
//        }
//
//        // д����ɣ����ñ�־λ
//        sd_write_in_progress = 0;
//        write_buffer = NULL;
//    }
//}

/**************************������SD�����*********************************/

/**************************��������̫�����***********************************/
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
        // printf("�ͻ����ѶϿ�����\r\n");
        return ERR_OK;
    }
    else
    {
        /* ���½��մ��� */
        tcp_recved(tpcb, p->tot_len);

        /* ����һ����̬����������������� */
        #define CMD_BUFFER_SIZE 64
        static char cmd_buffer[CMD_BUFFER_SIZE];
        static uint16_t cmd_buffer_index = 0;

        struct pbuf *q = p;
        while (q != NULL)
        {
            uint16_t len = q->len;
            uint8_t *payload = (uint8_t *)q->payload;

//            for (uint16_t i = 0; i < len; i++)
//            {
//                if (cmd_buffer_index < CMD_BUFFER_SIZE - 1)
//                {
//                    cmd_buffer[cmd_buffer_index++] = payload[i];
//
//                    /* ����Ƿ���յ����з�����ʾһ������Ľ��� */
//                    if (*(char*)(payload +i)== '\n')
//                    {
//                        cmd_buffer[cmd_buffer_index] = '\0';  // ȷ���ַ����Կ��ַ���β
//
//                        /* �������� */
//                        if (strncmp(cmd_buffer, "READ_SD", 7) == 0)
//                        {
//                            // ���ú������� SD ������
//                            printf("tcp_server_send_data\r\n");
//                            tcp_server_send_data(tpcb);
//                            
//                            // ����������
//                            cmd_buffer_index = 0;
//                            
//                            // �ͷŽ��յ��� pbuf
//                            pbuf_free(p);
//                            
//                            return ERR_OK;
//                        }
//                        else
//                        {
//                            // ����������������
//                        }
//
//                        // ����������
//                        cmd_buffer_index = 0;
//                    }
//                }
//                else
//                {
//                    // ��������������û�����
//                    cmd_buffer_index = 0;
//                }
//            }

            /* ����������ִ��ԭ�е����ݴ����߼� */
            // ��黺�����Ƿ����㹻�ռ�
            if (buffer_index + len <= BUFFER_SIZE)
            {
                memcpy((void *)(current_buffer + buffer_index), payload, len);
                buffer_index += len;
            }
            else
            {
                // ��ǰ������������׼���л�������

                // �ȴ� SD ��д�����
                // while (sd_write_in_progress);

                // �л�������
                __disable_irq();
                sd_write_in_progress = 1;
                if (current_buffer == buffer1)
                {
                    current_buffer = buffer2;
                    write_buffer = buffer1;
                }
                else
                {
                    current_buffer = buffer1;
                    write_buffer = buffer2;
                }

                buffer_index = 0;
                __enable_irq();

                // ��ʣ�������д���µĻ�����
                memcpy((void *)(current_buffer + buffer_index), payload, len);
                buffer_index += len;
            }

            q = q->next;
        }

        /* �ͷŽ��յ��� pbuf */
        pbuf_free(p);

        return ERR_OK;
    }
}

#define SD_READ_BUFFER_SIZE  (DMA_NUM_BLOCKS_TO_READ * BLOCK_SIZE)  // ÿ�ζ�ȡ�ͷ��͵��ֽ���
static err_t tcp_server_send_data(struct tcp_pcb *tpcb)
{
    uint32_t total_blocks = SD_CARD_CAPACITY_IN_BLOCKS;  // SD �����ܿ���
    uint32_t block = 0;
    uint8_t buffer_RX[DMA_NUM_BLOCKS_TO_READ * BLOCK_SIZE];
    err_t err;
    uint16_t mss = tcp_mss(tpcb);  // ��ȡ��ǰTCP���ӵ�MSS
    printf("��ǰMSSΪ: %u�ֽ�\n", mss);

    while (block < total_blocks)
    {
        // ���㱾�ζ�ȡ�Ŀ�������ֹ�����ܿ���
        uint32_t blocks_to_read = DMA_NUM_BLOCKS_TO_READ;
        if ((block + blocks_to_read) > total_blocks)
        {
            blocks_to_read = total_blocks - block;
        }

        // ���� DMA ��ȡ���ݵ� buffer_RX
        if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)buffer_RX, block, blocks_to_read) == HAL_OK)
        {
            // �ȴ���ȡ���
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER);

            // �����ȡ���ֽ���
            uint32_t bytes_read = blocks_to_read * BLOCK_SIZE;

            // �ֿ鷢������
            uint32_t bytes_sent = 0;
            while (bytes_sent < bytes_read)
            {
                // ��ȡ��ǰ TCP ���ͻ������Ĵ�С
                uint16_t max_len = tcp_sndbuf(tpcb);

                // ���㱾�ο��Է��͵�����ֽ�����ȷ�����ᳬ�� TCP ���ͻ������� MSS
                uint16_t len = bytes_read - bytes_sent;
                if (len > max_len) {
                    len = max_len;  // ȷ�����ᳬ�����û�������С
                }
                if (len > mss) {
                    len = mss;  // ȷ�����ᳬ�� MSS
                }

                // ��������
                err = tcp_write(tpcb, buffer_RX + bytes_sent, len, TCP_WRITE_FLAG_COPY);
                if (err != ERR_OK)
                {
                    printf("TCP д����󣬴�����룺%d\n", err);
                    return err;
                }

                // ���������͵����磬ȷ�����ͻ��������ͷ�
                err = tcp_output(tpcb);
                if (err != ERR_OK)
                {
                    printf("TCP ������󣬴�����룺%d\n", err);
                    return err;
                }

                bytes_sent += len;
            }

            block += blocks_to_read;  // ���¿��

            // ��ӡ���ȣ���ѡ��
            if (block % 10000 == 0)
            {
                printf("�ѷ��� %lu MB...\n", (block * BLOCK_SIZE) / (1024 * 1024));
            }
        }
        else
        {
            printf("��ȡ�� %lu ʧ�ܣ�״̬��%d\n", block, HAL_SD_GetError(&hsd));
            return ERR_VAL;  // ��ȡ���󣬷��ش�����
        }
    }

    // ���ݷ�����ɣ��ر� TCP ���ӣ�������Ҫ��
    // tcp_server_close(tpcb);

    printf("SD �����ݷ������\n");

    return ERR_OK;
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
    
//    struct tcp_pcb *tpcb;
//    uint16_t mss = tcp_mss(tpcb);
//    printf("LWIP��ʼ������ǰMSSΪ: %u�ֽ�\n", mss);

    // ȷ�� SD ���ѳ�ʼ���ɹ�
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        printf("SD ��δ׼���ã�������ѭ����\n");
    }
    else
    {
        printf("SD ����ʼ���ɹ���\n");
    }
    
    /* ��ʼ�� TCP ������ */
    tcp_server_init();
    generate_increasing_numbers(buffer_TX, sizeof(buffer_TX), 2) ;   
    sdio_write_test();
    HAL_Delay(100);
    sdio_read_speed_test();
    sdio_read_compare();
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        MX_LWIP_Process();
//        write_data_to_sd_card();
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
