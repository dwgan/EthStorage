#include "globals.h"
#include <stdlib.h> 


GlobalConfig_t g_Config = {
    .verNum = 0x06010101,             // �̼��汾
    .USART_BaudRate = 460800,           // ���ڲ�����
    .USART_Parity = UART_PARITY_NONE, // ����У��λ��Ĭ��Ϊ��У�� N
    .USART_StopBits = UART_STOPBITS_1, // ����ֹͣλ��Ĭ��Ϊ 1
    .workMode = READ,               // ����ģʽ��Ĭ��Ϊ��ȡ 0
    .dataInterface = 0,                // ���ݽӿڣ�Ĭ��Ϊ��̫�� 0
    .sdioAddrW = 15488,                // SDIO ��ǰ��д��ַ
    .sdioAddrR = 0                // SDIO ��ǰ��д��ַ
};
//GlobalConfig_t g_Config = {
//    .verNum = 0x06010101,             // �̼��汾
//    .USART_BaudRate = 460800,           // ���ڲ�����
//    .USART_Parity = UART_PARITY_NONE, // ����У��λ��Ĭ��Ϊ��У�� N
//    .USART_StopBits = UART_STOPBITS_1, // ����ֹͣλ��Ĭ��Ϊ 1
//    .workMode = WRITE,               // ����ģʽ��Ĭ��Ϊ��ȡ 0
//    .dataInterface = 0,                // ���ݽӿڣ�Ĭ��Ϊ��̫�� 0
//    .sdioAddrW = 0,                // SDIO ��ǰ��д��ַ
//    .sdioAddrR = 0                // SDIO ��ǰ��д��ַ
//};

// д�뵥����Ա�� Flash
void write_single_variable_to_flash(uint32_t flash_addr, uint32_t *data) {
    HAL_FLASH_Unlock();  // ���� Flash

    // ֱ��д�� 32 λ��ֵ��ָ���� Flash ��ַ
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flash_addr, *data) != HAL_OK) {
        // д��ʧ�ܴ���
        HAL_FLASH_Lock();
        printf("flash failed\n");
        return;
    }
    printf("flash succeed\n");

    HAL_FLASH_Lock();  // ���� Flash
}

// д�����õ� Flash
void write_config_to_flash(GlobalConfig_t *config)
{
    HAL_FLASH_Unlock();  // ���� Flash

    // ��������
    FLASH_EraseInitTypeDef eraseInitStruct;
    uint32_t sectorError = 0;

    eraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    eraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 3.3V
    eraseInitStruct.Sector = FLASH_SECTOR_11;  // ���� 11
    eraseInitStruct.NbSectors = 1;  // ֻ���� 1 ������

    if (HAL_FLASHEx_Erase(&eraseInitStruct, &sectorError) != HAL_OK) {
        // ����ʧ�ܴ���
        HAL_FLASH_Lock();
        return;
    }

    // д��ÿ�� 32 λ���ֵ� Flash
    uint32_t address = FLASH_USER_START_ADDR;
    uint32_t *data = (uint32_t *)config;  // ���ṹ��ת��Ϊ uint32_t ָ��
    for (int i = 0; i < sizeof(GlobalConfig_t) / 4; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, data[i]) != HAL_OK) {
            // д��ʧ�ܴ���
            HAL_FLASH_Lock();
            return;
        }
        address += 4;  // Flash ÿ��д�� 4 �ֽڣ�32 λ��
    }

    HAL_FLASH_Lock();  // ���� Flash
}

// �� Flash ��ȡ����
void read_config_from_flash(GlobalConfig_t *config) {
    uint32_t address = FLASH_USER_START_ADDR;
    uint32_t *data = (uint32_t *)config;

    // �� Flash ��ȡÿ�� 32 λ������
    for (int i = 0; i < sizeof(GlobalConfig_t) / 4; i++) {
        data[i] = *(__IO uint32_t *)address;
        address += 4;  // Flash ÿ�ζ�ȡ 4 �ֽڣ�32 λ��
    }
}
    
    
    
    
    