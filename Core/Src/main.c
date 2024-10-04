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
#define SD_CAPACITY 100 // 单位 MBytes
#define BLOCK_SIZE         512 // 一个块的字节数
#define DATA_SIZE_TO_WRITE 100*1024*1024 // 1MB = 1*1024*1024字节
#define DMA_NUM_BLOCKS_TO_WRITE 64 // 每一次DMA写入块的数量
#define DMA_NUM_BLOCKS_TO_READ  16 // 每一次DMA读出块的数量
#define NUM_TIMES_TO_WRITE DATA_SIZE_TO_WRITE/ (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_WRITE) // 总共要写入的次数
#define NUM_TIMES_TO_READ  DATA_SIZE_TO_WRITE/ (BLOCK_SIZE *DMA_NUM_BLOCKS_TO_READ) // 总共要写入的次数
#define SD_CARD_CAPACITY_IN_BLOCKS SD_CAPACITY*1024*1024/BLOCK_SIZE
#define BUFFER_SIZE         DMA_NUM_BLOCKS_TO_WRITE*BLOCK_SIZE         // 缓冲区大小

#define LAN8720_PHY_ADDRESS  0x00  // 根据你的硬件设计设置正确的PHY地址

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

volatile uint8_t *current_buffer = buffer1;    // 当前用于接收的缓冲区
volatile uint8_t *write_buffer = NULL;         // 当前用于写入SD卡的缓冲区

volatile uint32_t buffer_index = 0;            // 当前缓冲区的写入索引
volatile uint8_t buffer_full = 0;              // 标志当前缓冲区是否已满

volatile uint8_t sd_write_in_progress = 0;     // 标志是否正在进行SD卡写入


__ALIGN_BEGIN uint8_t buffer_TX[BUFFER_SIZE] __ALIGN_END;  // 4字节对齐，发送缓冲区
__ALIGN_BEGIN uint8_t buffer_RX[BLOCK_SIZE*DMA_NUM_BLOCKS_TO_READ] __ALIGN_END;  // 4字节对齐，接收缓冲区

volatile uint8_t sd_write_complete = 0; // 写入完成标志
volatile uint8_t sd_read_complete = 0;  // 读取完成标志

extern struct netif gnetif;
extern ETH_HandleTypeDef heth;

int write_enable = 1;
int read_enable = 1;
int read_done = 1;
// 定义全局 TCP 控制块变量
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

/**************************以下是SD卡相关*********************************/
///* DMA回调函数 */
//void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
//{
//    sd_write_complete = 1;
////    sd_write_in_progress = 0;
//}
// 写入起始地址（以 block 为单位）
uint32_t write_address = 0;
uint32_t read_address = 0;

// 开始写入数据
void Start_Write(void) {
    if (HAL_SD_WriteBlocks_DMA(&hsd, buffer_TX, write_address, DMA_NUM_BLOCKS_TO_WRITE) != HAL_OK) {
        // 写入错误处理
        printf("开启中断写入错误\n");
    }
    HAL_SD_GetCardState(&hsd);
}
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
    // 更新写入地址
//    write_address += DMA_NUM_BLOCKS_TO_WRITE;

    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

    // 检查是否有更多数据需要写入
    static uint32_t num=0;
//    if (num++ < DATA_SIZE_TO_WRITE/ DMA_NUM_BLOCKS_TO_WRITE/BLOCK_SIZE) {
//    }
//    printf("0x%4.x\n", HAL_SD_GetCardState(hsd));
    HAL_SD_GetCardState(hsd);
    write_enable = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
    printf("SDIO 传输错误！\n");
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
int compare_buffers(unsigned char *buffer1, unsigned char *buffer2, int size) {
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
    HAL_SD_GetCardState(hsd);
    
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    if (!read_enable)
    {
        read_enable = 1;
        read_done = 1;
    }
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

void sdio_write_test()
{    
    // 记录写入开始时间
    printf("开始写入数据到 SD 卡...\n");
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
                        printf("中断写入错误:%d\n", status);
                        continue;
                    }
                    write_address += DMA_NUM_BLOCKS_TO_WRITE;
                    write_enable = 0;
                    
                    if (write_address % (DMA_NUM_BLOCKS_TO_WRITE*2048) == 0) {
                        printf("已写入 %d MB...\r\n", (write_address * BLOCK_SIZE) / (1024 * 1024));
                    }
                }
            }
            
        }
        else
        {
            break;
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
}

void sdio_read_speed_test()
{
    
    /*--------------------------SD卡读取速度测试--------------------------*/
    // 记录读取开始时间
    printf("开始读取数据从 SD 卡...\n");

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
                        printf("中断读取错误:%d\nBlock: %d", status, read_address);
                        continue;
                    }
                    read_address += DMA_NUM_BLOCKS_TO_READ;
                    read_enable = 0;
                    
//                    if (write_address % (DMA_NUM_BLOCKS_TO_READ*2048) == 0) {
//                        printf("已读取 %d MB...\r\n", (read_address * BLOCK_SIZE) / (1024 * 1024));
//                    }
                }
            }
            
        }
        else
        {
            break;
        }
    }
    // 记录读取结束时间
    uint32_t end_time = HAL_GetTick();
    
    // 计算总读取时间（秒）
    float elapsed_time_sec = (end_time - start_time) / 1000.0f;
    
    // 计算读取速度（MB/s）
    float write_speed = DATA_SIZE_TO_WRITE / (1024.0f * 1024.0f) / elapsed_time_sec;
    
    printf("数据读取完成！\n");
    printf("总读取时间: %.2f 秒\n", elapsed_time_sec);
    printf("读取速度: %.2f MB/s\n", write_speed);
    
}

void sdio_read_compare()
{
    /*--------------------------SD卡读测试--------------------------*/
    printf("开始读取数据从 SD 卡...\n");
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
                        printf("数据验证失败在块 %d\r\n", read_address-DMA_NUM_BLOCKS_TO_READ);
                        while (1); // 进入死循环，数据错误
                    }
                    else
                    {
//                        printf("数据验证成功在块 %d\r\n", read_address-DMA_NUM_BLOCKS_TO_READ);
                    }
                    read_done = 0;
                }
                //                if (!(HAL_SD_GetCardState(&hsd) & HAL_SD_CARD_TRANSFER))
                if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1 && HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == 1)
                {
                    int status = HAL_SD_ReadBlocks_DMA(&hsd, buffer_RX, read_address, DMA_NUM_BLOCKS_TO_READ);
                    if (status != HAL_OK) {
                        printf("中断读取错误:%d\nBlock: %d", status, read_address);
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
    printf("数据读取和验证完成，写入和读取的数据一致！\n");
}

//void write_data_to_sd_card()
//{
//    static uint32_t BlockAdd = 1000;  // SD卡的起始块地址
//    HAL_StatusTypeDef status;
//
//    if (sd_write_in_progress && write_buffer != NULL)
//    {
//        uint32_t data_length = BUFFER_SIZE;  // 假设缓冲区已满
//        uint8_t retry = 0;
//
//        // 计算需要写入的块数
//        uint32_t blocks_to_write = data_length / BLOCK_SIZE;
//
//        // 写入SD卡
//        do
//        {
//            status = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)write_buffer, BlockAdd, blocks_to_write);
//            if (status == HAL_OK)
//            {
//                // 等待写入完成
//                while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {}
//                BlockAdd += blocks_to_write;  // 更新块地址
//                break;
//            }
//            else
//            {
//                printf("写入块 %lu 失败，状态：%d。重试 %d/%d\r\n",
//                       BlockAdd, HAL_SD_GetError(&hsd), retry + 1, MAX_RETRIES);
//                retry++;
//                HAL_Delay(RETRY_DELAY_MS);
//            }
//        } while (retry < 1);
//
//        if (status != HAL_OK)
//        {
//            printf("写入块 %lu 失败，达到最大重试次数，跳过此块。\n", BlockAdd);
//            BlockAdd += blocks_to_write;  // 跳过有问题的块
//        }
//
//        // 写入完成，重置标志位
//        sd_write_in_progress = 0;
//        write_buffer = NULL;
//    }
//}

/**************************以上是SD卡相关*********************************/

/**************************以下是以太网相关***********************************/
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
        // printf("客户端已断开连接\r\n");
        return ERR_OK;
    }
    else
    {
        /* 更新接收窗口 */
        tcp_recved(tpcb, p->tot_len);

        /* 定义一个静态缓冲区用于命令解析 */
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
//                    /* 检查是否接收到换行符，表示一条命令的结束 */
//                    if (*(char*)(payload +i)== '\n')
//                    {
//                        cmd_buffer[cmd_buffer_index] = '\0';  // 确保字符串以空字符结尾
//
//                        /* 处理命令 */
//                        if (strncmp(cmd_buffer, "READ_SD", 7) == 0)
//                        {
//                            // 调用函数发送 SD 卡数据
//                            printf("tcp_server_send_data\r\n");
//                            tcp_server_send_data(tpcb);
//                            
//                            // 清空命令缓冲区
//                            cmd_buffer_index = 0;
//                            
//                            // 释放接收到的 pbuf
//                            pbuf_free(p);
//                            
//                            return ERR_OK;
//                        }
//                        else
//                        {
//                            // 处理其他命令或忽略
//                        }
//
//                        // 清空命令缓冲区
//                        cmd_buffer_index = 0;
//                    }
//                }
//                else
//                {
//                    // 缓冲区溢出，重置缓冲区
//                    cmd_buffer_index = 0;
//                }
//            }

            /* 如果不是命令，执行原有的数据处理逻辑 */
            // 检查缓冲区是否有足够空间
            if (buffer_index + len <= BUFFER_SIZE)
            {
                memcpy((void *)(current_buffer + buffer_index), payload, len);
                buffer_index += len;
            }
            else
            {
                // 当前缓冲区已满，准备切换缓冲区

                // 等待 SD 卡写入完成
                // while (sd_write_in_progress);

                // 切换缓冲区
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

                // 将剩余的数据写入新的缓冲区
                memcpy((void *)(current_buffer + buffer_index), payload, len);
                buffer_index += len;
            }

            q = q->next;
        }

        /* 释放接收到的 pbuf */
        pbuf_free(p);

        return ERR_OK;
    }
}

#define SD_READ_BUFFER_SIZE  (DMA_NUM_BLOCKS_TO_READ * BLOCK_SIZE)  // 每次读取和发送的字节数
static err_t tcp_server_send_data(struct tcp_pcb *tpcb)
{
    uint32_t total_blocks = SD_CARD_CAPACITY_IN_BLOCKS;  // SD 卡的总块数
    uint32_t block = 0;
    uint8_t buffer_RX[DMA_NUM_BLOCKS_TO_READ * BLOCK_SIZE];
    err_t err;
    uint16_t mss = tcp_mss(tpcb);  // 获取当前TCP连接的MSS
    printf("当前MSS为: %u字节\n", mss);

    while (block < total_blocks)
    {
        // 计算本次读取的块数，防止超出总块数
        uint32_t blocks_to_read = DMA_NUM_BLOCKS_TO_READ;
        if ((block + blocks_to_read) > total_blocks)
        {
            blocks_to_read = total_blocks - block;
        }

        // 启动 DMA 读取数据到 buffer_RX
        if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)buffer_RX, block, blocks_to_read) == HAL_OK)
        {
            // 等待读取完成
            while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER);

            // 计算读取的字节数
            uint32_t bytes_read = blocks_to_read * BLOCK_SIZE;

            // 分块发送数据
            uint32_t bytes_sent = 0;
            while (bytes_sent < bytes_read)
            {
                // 获取当前 TCP 发送缓冲区的大小
                uint16_t max_len = tcp_sndbuf(tpcb);

                // 计算本次可以发送的最大字节数，确保不会超过 TCP 发送缓冲区和 MSS
                uint16_t len = bytes_read - bytes_sent;
                if (len > max_len) {
                    len = max_len;  // 确保不会超过可用缓冲区大小
                }
                if (len > mss) {
                    len = mss;  // 确保不会超过 MSS
                }

                // 发送数据
                err = tcp_write(tpcb, buffer_RX + bytes_sent, len, TCP_WRITE_FLAG_COPY);
                if (err != ERR_OK)
                {
                    printf("TCP 写入错误，错误代码：%d\n", err);
                    return err;
                }

                // 将数据推送到网络，确保发送缓冲区被释放
                err = tcp_output(tpcb);
                if (err != ERR_OK)
                {
                    printf("TCP 输出错误，错误代码：%d\n", err);
                    return err;
                }

                bytes_sent += len;
            }

            block += blocks_to_read;  // 更新块号

            // 打印进度（可选）
            if (block % 10000 == 0)
            {
                printf("已发送 %lu MB...\n", (block * BLOCK_SIZE) / (1024 * 1024));
            }
        }
        else
        {
            printf("读取块 %lu 失败，状态：%d\n", block, HAL_SD_GetError(&hsd));
            return ERR_VAL;  // 读取错误，返回错误码
        }
    }

    // 数据发送完成，关闭 TCP 连接（根据需要）
    // tcp_server_close(tpcb);

    printf("SD 卡数据发送完成\n");

    return ERR_OK;
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
/**************************以上是以太网相关*********************************/

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
//    printf("LWIP初始化，当前MSS为: %u字节\n", mss);

    // 确保 SD 卡已初始化成功
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) {
        printf("SD 卡未准备好，进入死循环。\n");
    }
    else
    {
        printf("SD 卡初始化成功！\n");
    }
    
    /* 初始化 TCP 服务器 */
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
