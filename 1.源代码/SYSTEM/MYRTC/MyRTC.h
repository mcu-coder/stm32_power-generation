#ifndef __MYRTC_H
#define __MYRTC_H

uint8_t OPEN_HOUR =15; //定时开启时
uint8_t OPEN_MINUTE =15;//定时开启分
uint8_t CLOSE_HOUR =15;//定时关闭时
uint8_t CLOSE_MINUTE =16;//定时关闭分
extern uint16_t MyRTC_Time[];

void MyRTC_Init(void);
void MyRTC_SetTime(void);
void MyRTC_ReadTime(void);

#endif

