#ifndef UART_H_
#define UART_H_

extern void serialInit(void);
extern void serialSendChar(double* value, unsigned char newLine);

#define putNewLine 1
#define noNewLine 0

#define gyroXaxis 0xAA
#define gyroYaxis 0xAB
#define gyroZaxis 0xAC

#define RxChannel1 0xBA
#define RxChannel2 0xBB
#define RxChannel3 0xBC
#define RxChannel4 0xBD

#endif
