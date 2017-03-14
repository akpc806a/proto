#ifndef PS2INTERFACE_H
#define PS2INTERFACE_H

#include <stdint.h>

// callback for driving GPIO pin
typedef void (*T_PSInterface_GpioDriveCallback) (uint8_t value);

// callback for reading GPIO pin
typedef uint8_t (*T_PSInterface_GpioGetCallback) (void);

// timer restart callback
typedef void (*T_PSInterface_TimerCallback) (void);

// receiver states
typedef enum
{
  PS2_STATE_START,
  PS2_STATE_DATA,
  PS2_STATE_PARITY,
  PS2_STATE_STOP
} PS_RX_States;

// byte queue from template defenition
#define QUEUE_SIZE 64
typedef unsigned char byte;
#include "queue.h"
CREATE_QUEUE_TYPE_H(byte)

// PS2 interfae instance
typedef struct
{
  PS_RX_States RxState;
  uint8_t BitCounter;
  uint8_t RxData;
  uint8_t TimeOut;
  uint8_t Error;
  
  byteQueue RxQueue;
  
  T_PSInterface_GpioDriveCallback SCLK_Drive;
  T_PSInterface_GpioDriveCallback DATA_Drive;
  T_PSInterface_GpioGetCallback SCLK_Get;
  T_PSInterface_GpioGetCallback DATA_Get;
  
  T_PSInterface_TimerCallback TimerRestart;
  
} T_PS2Interface;  

// initialization of PS/2 interface
int PS2Interface_Init(T_PS2Interface* instance, T_PSInterface_GpioDriveCallback SCLK_Drive_, T_PSInterface_GpioDriveCallback DATA_Drive_, T_PSInterface_GpioGetCallback SCLK_Get_, T_PSInterface_GpioGetCallback DATA_Get_, T_PSInterface_TimerCallback TimerRestart_);

// should be called from EXTI interrupt to process received bit at rising edge
void PS2Interface_ProcessRx(T_PS2Interface* instance, uint8_t bit);

// called from timer interrupt to distinguish pause between two bytes 
void PS2Interface_RxTimeOutElapsed(T_PS2Interface* instance);

// put bus into host mode and transmit 8 bit data
void PS2Interface_Send(T_PS2Interface* instance, uint8_t data);

#endif
