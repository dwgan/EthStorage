#ifndef GLOBALS_H_
#define GLOBALS_H_

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stm32f4xx.h>
#include <stdio.h>
#include <stdint.h>
	
#define FLASH_USER_START_ADDR   ((uint32_t)0x080E0000)   // Flash ���� 11 ��ʼ��ַ
#define FLASH_USER_END_ADDR     ((uint32_t)0x080FFFFF)   // Flash ���� 11 ������ַ


    typedef enum{
        WRITE=0,
        READ=1
    }WORKMODE;
    
#pragma pack(4)
    // �������ýṹ
    typedef struct {
        uint32_t verNum; //verNum
        uint16_t chipID;
        uint32_t USART_BaudRate;
        uint16_t USART_Parity;
        uint16_t USART_StopBits;
        uint8_t workMode;  // 0=��ȡ��1=�洢
        uint8_t dataInterface;  // 0=��̫����1=����
        uint32_t sdioAddrW;
        uint32_t sdioAddrR;
    } GlobalConfig_t;
    
#pragma pack()
    extern GlobalConfig_t g_Config;
void write_config_to_flash(GlobalConfig_t *config);
void read_config_from_flash(GlobalConfig_t *config);
void write_single_variable_to_flash(uint32_t flash_addr, uint32_t *data);
    
#ifdef __cplusplus
}
#endif

#endif /* GLOBALS_H_ */
