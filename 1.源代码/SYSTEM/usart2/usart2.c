#include "usart2.h"
#include "string.h"

static uint8_t USART2_RxBuffer[USART2_RX_BUF_SIZE];  // 接收缓冲区
static volatile uint16_t USART2_RxIndex = 0;          // 接收索引
static volatile bool USART2_RxFlag = false;           // 接收完成标志

/**
  * @brief  初始化USART2
  * @param  baudrate: 波特率（如9600、115200）
  */
void USART2_Init(uint32_t baudrate) {
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    // 1. 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // 2. 配置GPIO（PA2-TX, PA3-RX）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;  // TX-复用推挽输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;  // RX-浮空输入
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. 配置USART2参数
    USART_InitStruct.USART_BaudRate = baudrate;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &USART_InitStruct);

    // 4. 配置中断（接收中断）
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    // 5. 使能USART2
    USART_Cmd(USART2, ENABLE);
}

/**
  * @brief  USART2中断服务函数
  */
void USART2_IRQHandler(void) {
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
        uint8_t data = USART_ReceiveData(USART2);
        if (USART2_RxIndex < USART2_RX_BUF_SIZE) {
            USART2_RxBuffer[USART2_RxIndex++] = data;
            USART2_RxFlag = true;  // 设置接收标志
        } else {
            USART2_RxIndex = 0;    // 缓冲区溢出则重置
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/**
  * @brief  发送单字节数据
  * @param  data: 待发送的字节
  */
void USART2_SendByte(uint8_t data) {
    USART_SendData(USART2, data);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);  // 等待发送完成
}

/**
  * @brief  发送字符串
  * @param  str: 待发送的字符串（以'\0'结尾）
  */
void USART2_SendString(const char* str) {
    while (*str) {
        USART2_SendByte(*str++);
    }
}

/**
  * @brief  发送字节数组
  * @param  arr: 数组指针
  * @param  len: 数组长度
  */
void USART2_SendArray(uint8_t* arr, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {
        USART2_SendByte(arr[i]);
    }
}

// 其他辅助函数
bool USART2_GetReceivedFlag(void) { return USART2_RxFlag; }
uint16_t USART2_GetReceivedLength(void) { return USART2_RxIndex; }
uint8_t* USART2_GetRxBuffer(void) { return (uint8_t*)USART2_RxBuffer; }
void USART2_ClearReceivedFlag(void) { USART2_RxFlag = false; USART2_RxIndex = 0; }