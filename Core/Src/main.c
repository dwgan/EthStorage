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
//#define BUFFER_SIZE_W         DMA_NUM_BLOCKS_TO_WRITE*BLOCK_SIZE         // 缓冲区大小
#define BUFFER_SIZE_W         600// 缓冲区大小
#define BUFFER_SIZE_R         DMA_NUM_BLOCKS_TO_READ*BLOCK_SIZE         // 缓冲区大小
#define BUFFER_DEPTH 2
#define TEMP_BUFF_SIZE 2048

#define LAN8720_PHY_ADDRESS  0x00  // 根据你的硬件设计设置正确的PHY地址

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
    uint8_t write_prt;//当前正在写入的buff
    uint8_t write_prt_next;//当前正在写入的buff
    uint8_t read_prt;//当前正在写入的buff
    uint8_t read_prt_next;//当前正在写入的buff
    uint32_t index[BUFFER_DEPTH];//写入指针，指示当前buff字节位置
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
    .buff_state = {READY2WRITE, READY2WRITE}, // 初始化所有缓冲区状态
    .buff = {0}
};
uint32_t last_tcp_receive_time = 0; // 记录上一次接收到TCP数据的时间戳（毫秒）


SD2TCP_Buff_t sd2tcp_buff={0};
uint32_t temp_buff_len = 0;
uint8_t temp_buff[TEMP_BUFF_SIZE];
struct tcp_pcb *global_pcb = NULL;        // 用于暂停和恢复 TCP 接收

int write_enable = 1;
int read_enable = 1;
int read_done = 1;
static struct tcp_pcb *tcp_server_pcb;
uint32_t write_address = 0;
uint32_t read_address = 0;
//volatile uint8_t current_buff_index = 0;      // 当前正在写入的缓冲区索引（0 或 1）
//volatile uint8_t sd_write_in_progress = 0;    // 标志 SD 卡写入是否正在进行
#pragma pack()
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void tcp_server_error(void *arg, err_t err);
static void tcp_server_close(struct tcp_pcb *tpcb);
//static err_t tcp_server_send_data(struct tcp_pcb *tpcb);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**************************以下是SD卡相关*********************************/


// 开始写入数据
void Start_Write(void) {
    if (HAL_SD_WriteBlocks_DMA(&hsd, tcp2sd_buff.buff[0], write_address, DMA_NUM_BLOCKS_TO_WRITE) != HAL_OK) {
        // 写入错误处理
        printf("开启中断写入错误\n");
    }
    HAL_SD_GetCardState(&hsd);
}
void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd)
{
//    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);

//    HAL_SD_GetCardState(hsd);
    write_enable = 1;
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd)
{
//    printf("SDIO 传输错误！\n");
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
    // 在这里处理数据，例如将数据写入其他存储器，或进行计算
    static int mismatch_index;
    mismatch_index = compare_buffers(tcp2sd_buff.buff[0]+(block * BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ)%BUFFER_SIZE_W, buffer, BLOCK_SIZE * DMA_NUM_BLOCKS_TO_READ);
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
                    int status = HAL_SD_WriteBlocks_DMA(&hsd, tcp2sd_buff.buff[0], write_address, DMA_NUM_BLOCKS_TO_WRITE);
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
                    int status = HAL_SD_ReadBlocks_DMA(&hsd, sd2tcp_buff.buff_r[0] + (read_address * BLOCK_SIZE)%sizeof(sd2tcp_buff.buff_r[0]), read_address, DMA_NUM_BLOCKS_TO_READ);
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
                    int status = HAL_SD_ReadBlocks_DMA(&hsd, sd2tcp_buff.buff_r[0], read_address, DMA_NUM_BLOCKS_TO_READ);
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
/**************************以上是SD卡相关*********************************/

/**************************以下是缓存相关***********************************/
void reset_buffers()
{
    // 重置写入和读取指针
    tcp2sd_buff.write_prt = 0;
    tcp2sd_buff.read_prt = 0;

    // 重置每个缓冲区的状态和索引
    for (int i = 0; i < BUFFER_DEPTH; i++)
    {
        tcp2sd_buff.index[i] = 0;
        tcp2sd_buff.buff_state[i] = READY2WRITE;
    }

    // 清空缓冲区内容（可选）
    memset(tcp2sd_buff.buff, 0, sizeof(tcp2sd_buff.buff));

    // 重置全局TCP控制块指针
    global_pcb = NULL;
}

void write_data_to_uart()
{
    // 如果当前读取缓冲区有数据
    if (tcp2sd_buff.buff_state[tcp2sd_buff.read_prt] == READY2READ)
    {
        // 通过 UART 传输数据
        HAL_UART_Transmit(&huart1, tcp2sd_buff.buff[tcp2sd_buff.read_prt], tcp2sd_buff.index[tcp2sd_buff.read_prt], 1000);
        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {}  // 等待 UART 传输完成

        // 更新缓冲区状态为READY2WRITE
        tcp2sd_buff.buff_state[tcp2sd_buff.read_prt] = READY2WRITE;
        tcp2sd_buff.index[tcp2sd_buff.read_prt] = 0;

        // 切换到下一个读取缓冲区
        tcp2sd_buff.read_prt = tcp2sd_buff.read_prt_next;
        tcp2sd_buff.read_prt_next = (tcp2sd_buff.read_prt_next + 1) % BUFFER_DEPTH;

        // **如果 TCP 接收被暂停，则恢复**
        if (global_pcb != NULL)
        {
            HAL_UART_Transmit(&huart1, temp_buff, temp_buff_len, 1000);
            while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET) {}  // 等待 UART 传输完成
            // 重置 global_pcb
            struct tcp_pcb *temp_pcb = global_pcb;
            global_pcb = NULL;

            // 重新调用 tcp_recved，通知 TCP 可以继续发送数据
            tcp_recved(temp_pcb, 1);

            // 重新设置接收回调函数
            tcp_recv(temp_pcb, tcp_server_recv);
        }
    }
}


void flush_data_to_uart()
{
    // 检查TCP数据接收是否超时
    uint32_t current_time = HAL_GetTick();
    if ((current_time - last_tcp_receive_time) > 1000)
    {
        // 检查是否已经有数据在缓冲区中
        if (tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] == READY2WRITE && tcp2sd_buff.index[tcp2sd_buff.write_prt] > 0)
        {
            // 将当前写缓冲区标记为 READY2READ，等待发送
            tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] = READY2READ;
        }
        
        // 处理所有剩余的缓冲区数据
        while (tcp2sd_buff.buff_state[tcp2sd_buff.read_prt] == READY2READ)
        {
            // 输出缓冲区数据
            write_data_to_uart();
        }
        
        // 如果需要，可以关闭TCP连接
        if (global_pcb != NULL)
        {
            tcp_server_close(global_pcb);
            global_pcb = NULL;
        }
        
        // 重置最后接收时间戳，防止重复进入超时处理
        last_tcp_receive_time = HAL_GetTick();
    }
}

/**************************以上是缓存相关***********************************/
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

    // 重置最后接收时间戳
    last_tcp_receive_time = HAL_GetTick();
    
//    printf("客户端已连接\r\n");
//    reset_buffers();  // 初始化缓冲区和相关变量，确保第一次连接时状态一致
    return ERR_OK;
}

static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (p == NULL)
    {
        last_tcp_receive_time = HAL_GetTick();
        
        // 连接关闭，处理必要的剩余数据
        tcp_server_close(tpcb);
        return ERR_OK;
    }
    else
    {
        struct pbuf *q = p;
        uint32_t total_copied_len = 0; // 总共复制的数据长度

        while (q != NULL)
        {
            last_tcp_receive_time = HAL_GetTick();
            
            uint16_t len = q->len;
            uint8_t *payload = (uint8_t *)q->payload;

            uint8_t *data_ptr = payload;
            uint32_t data_len = len; // 当前pbuf中剩余的数据长度
            uint16_t copy_len;
            
            while (data_len > 0)
            {
                // 计算当前写缓冲区剩余空间
                uint32_t buffer_space = BUFFER_SIZE_W - tcp2sd_buff.index[tcp2sd_buff.write_prt];

                // 如果当前写缓冲区已满，切换到下一个缓冲区
                if (buffer_space == 0)
                {
                    // 标记当前缓冲区为READY2READ
                    tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] = READY2READ;
                    
                    // 切换到下一个缓冲区
                    tcp2sd_buff.write_prt = tcp2sd_buff.write_prt_next;
                    tcp2sd_buff.write_prt_next = (tcp2sd_buff.write_prt_next + 1) % BUFFER_DEPTH;
                    
                    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_15);
                    // 检查下一个缓冲区是否可写
                    if (tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] != READY2WRITE)
                    {
                        // 没有可用的缓冲区，暂停TCP接收
                        temp_buff_len += len-copy_len;
                        memcpy(temp_buff+temp_buff_len, data_ptr+copy_len,temp_buff_len); 
                        global_pcb = tpcb;
                        
                        // 释放pbuf
                        pbuf_free(p);
                        
                        return ERR_OK;
                    }
                    else
                    {
                        // 重置新缓冲区的写入索引
                        tcp2sd_buff.index[tcp2sd_buff.write_prt] = 0;
                        buffer_space = BUFFER_SIZE_W;
                    }
                }

                // 计算本次可以复制的数据长度
                copy_len = data_len > buffer_space ? buffer_space : data_len;

                // 复制数据到缓冲区
                memcpy(&tcp2sd_buff.buff[tcp2sd_buff.write_prt][tcp2sd_buff.index[tcp2sd_buff.write_prt]], data_ptr, copy_len);

                // 更新索引和指针
                tcp2sd_buff.index[tcp2sd_buff.write_prt] += copy_len;
                data_ptr += copy_len;
                data_len -= copy_len;
                total_copied_len += copy_len;

                // 如果当前缓冲区已满，标记为READY2READ并切换缓冲区
                if (tcp2sd_buff.index[tcp2sd_buff.write_prt] >= BUFFER_SIZE_W)
                {
                    tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] = READY2READ;
                    tcp2sd_buff.write_prt = tcp2sd_buff.write_prt_next;
                    tcp2sd_buff.write_prt_next = (tcp2sd_buff.write_prt_next + 1) % BUFFER_DEPTH;

                    // 检查下一个缓冲区是否可写
                    if (tcp2sd_buff.buff_state[tcp2sd_buff.write_prt] != READY2WRITE)
                    {
//                        temp_buff_len += len-copy_len;
//                        memcpy(temp_buff+temp_buff_len, data_ptr+copy_len,temp_buff_len); 
                        // 没有可用的缓冲区，暂停TCP接收
                        global_pcb = tpcb;

                        // 释放pbuf
                        pbuf_free(p);

                        return ERR_OK;
                    }
                    else
                    {
                        // 重置新缓冲区的写入索引
                        tcp2sd_buff.index[tcp2sd_buff.write_prt] = 0;
//                        buffer_space = BUFFER_SIZE_W;
                    }
                }
            }

            // 处理下一个pbuf
            q = q->next;
        }

        // 确认已接收的数据量
        tcp_recved(tpcb, total_copied_len);

        // 释放pbuf
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
    
    struct tcp_pcb *tpcb;
    uint16_t mss = tcp_mss(tpcb);
    printf("LWIP初始化，当前MSS为: %u字节\n", mss);

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
//    generate_increasing_numbers(tcp2sd_buff.buff[0], sizeof(tcp2sd_buff.buff[0]), 2) ;   
//    sdio_write_test();
//    HAL_Delay(100);
//    sdio_read_speed_test();
//    sdio_read_compare();
//    reset_buffers();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        MX_LWIP_Process();
        write_data_to_uart();
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
