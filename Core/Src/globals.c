#include "globals.h"
#include <stdlib.h> 


GlobalConfig_t g_Config = {
    .verNum = 0x06010101,             // 固件版本
    .USART_BaudRate = 460800,           // 串口波特率
    .USART_Parity = UART_PARITY_NONE, // 串口校验位，默认为无校验 N
    .USART_StopBits = UART_STOPBITS_1, // 串口停止位，默认为 1
    .workMode = READ,               // 操作模式，默认为读取 0
    .dataInterface = 0,                // 数据接口，默认为以太网 0
    .sdioAddrW = 15488,                // SDIO 当前的写地址
    .sdioAddrR = 0                // SDIO 当前的写地址
};
//GlobalConfig_t g_Config = {
//    .verNum = 0x06010101,             // 固件版本
//    .USART_BaudRate = 460800,           // 串口波特率
//    .USART_Parity = UART_PARITY_NONE, // 串口校验位，默认为无校验 N
//    .USART_StopBits = UART_STOPBITS_1, // 串口停止位，默认为 1
//    .workMode = WRITE,               // 操作模式，默认为读取 0
//    .dataInterface = 0,                // 数据接口，默认为以太网 0
//    .sdioAddrW = 0,                // SDIO 当前的写地址
//    .sdioAddrR = 0                // SDIO 当前的写地址
//};

// 写入单个成员到 Flash
void write_single_variable_to_flash(uint32_t flash_addr, uint32_t *data) {
    HAL_FLASH_Unlock();  // 解锁 Flash

    // 直接写入 32 位的值到指定的 Flash 地址
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr, *data) != HAL_OK) {
        // 写入失败处理
        HAL_FLASH_Lock();
        printf("flash failed\n");
        return;
    }
    printf("flash succeed\n");

    HAL_FLASH_Lock();  // 锁定 Flash
}

// 写入配置到 Flash
void write_config_to_flash(GlobalConfig_t *config)
{
    HAL_FLASH_Unlock();  // 解锁 Flash

    // 擦除扇区
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t sectorError = 0;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 3.3V
    eraseInitStruct.Sector = FLASH_SECTOR_11;  // 扇区 11
    eraseInitStruct.NbSectors = 1;  // 只擦除 1 个扇区

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError) != HAL_OK) {
        // 擦除失败处理
        HAL_FLASH_Lock();
        return;
    }

    // 写入每个 32 位的字到 Flash
    uint32_t address = FLASH_USER_START_ADDR;
    uint32_t *data = (uint32_t *)config;  // 将结构体转换为 uint32_t 指针
    for (int i = 0; i < sizeof(GlobalConfig_t) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]) != HAL_OK) {
            // 写入失败处理
            HAL_FLASH_Lock();
            return;
        }
        address += 4;  // Flash 每次写入 4 字节（32 位）
    }

    HAL_FLASH_Lock();  // 锁定 Flash
}

// 从 Flash 读取配置
void read_config_from_flash(GlobalConfig_t *config) {
    uint32_t address = FLASH_USER_START_ADDR;
    uint32_t *data = (uint32_t *)config;

    // 从 Flash 读取每个 32 位的数据
    for (int i = 0; i < sizeof(GlobalConfig_t) / 4; i++) {
        data[i] = *(__IO uint32_t *)address;
        address += 4;  // Flash 每次读取 4 字节（32 位）
    }
}
    
    
    
    
    