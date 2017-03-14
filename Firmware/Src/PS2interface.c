#include "PS2Interface.h"

// include functions to create queue of bytes, i.e. byteQueue_XX
CREATE_QUEUE_TYPE_C(byte)

// initialization of PS/2 interface
// paramters are: instance -- pointer to data structure, and callbacks to hardware-dependant functions
int PS2Interface_Init(T_PS2Interface* instance, T_PSInterface_GpioDriveCallback SCLK_Drive_, T_PSInterface_GpioDriveCallback DATA_Drive_, T_PSInterface_GpioGetCallback SCLK_Get_, T_PSInterface_GpioGetCallback DATA_Get_, T_PSInterface_TimerCallback TimerRestart_)
{
  if (instance == 0) return 0;
  if (SCLK_Drive_ == 0) return 0;
  if (DATA_Drive_ == 0) return 0;
  if (TimerRestart_ == 0) return 0;
  
  instance->RxState = PS2_STATE_START;
  instance->BitCounter = 0;
  instance->RxData = 0;
  instance->TimeOut = 0;
  instance->Error = 0;
  
  instance->SCLK_Drive = SCLK_Drive_;
  instance->DATA_Drive = DATA_Drive_;
  instance->SCLK_Get = SCLK_Get_;
  instance->DATA_Get = DATA_Get_;
  instance->TimerRestart = TimerRestart_;
  
  return 1;
}

// parity computation -- based on http://stackoverflow.com/a/21618038
static uint8_t compute_parity(uint8_t x)
{
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return (~x) & 1;
}

// should be called from EXTI interrupt to process received bit at rising edge
void PS2Interface_ProcessRx(T_PS2Interface* instance, uint8_t bit)
{
  PS_RX_States state;
  //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    
  state = instance->RxState;
  
  if (instance->TimeOut || state == PS2_STATE_START)
  {
    if (bit == 0)
    {
      state = PS2_STATE_DATA;
      instance->BitCounter = 0;
      instance->RxData = 0;
      
      //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    }
  }
  else
  if (state == PS2_STATE_DATA)
  {
    if (bit)
      instance->RxData |= (1 << instance->BitCounter);
    instance->BitCounter++;
    
    if (instance->BitCounter == 8)
    {
      state = PS2_STATE_PARITY;
      
      
      //HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    }
  }
  else
  if (state == PS2_STATE_PARITY)
  {
    if (bit)
    {
      if (compute_parity(instance->RxData))
        byteQueue_Put(&(instance->RxQueue), instance->RxData);
      else
        instance->Error = 1; // TODO: specific code?
    }
    else
    {
      if (! compute_parity(instance->RxData))
        byteQueue_Put(&(instance->RxQueue), instance->RxData);
      else
        instance->Error = 1; // TODO: specific code?
    }
    
    state = PS2_STATE_STOP;
  }
  else
  if (state == PS2_STATE_STOP)
  {
    state = PS2_STATE_START;
  }
  
  instance->TimeOut = 0;
  (*(instance->TimerRestart))(); // restart time-out timer
  
  
  instance->RxState = state;
}

// called from timer interrupt to distinguish pause between two bytes 
void PS2Interface_RxTimeOutElapsed(T_PS2Interface* instance)
{
  instance->TimeOut = 1;
}

// put bus into host mode and transmit 8 bit data (blocking function)
void PS2Interface_Send(T_PS2Interface* instance, uint8_t data)
{
  int i;
  // http://www.computer-engineering.org/ps2protocol/
  (*(instance->SCLK_Drive))(0); // 1)   Bring the Clock line low for at least 100 microseconds
  for (i = 0; i < 1000/*0*/; i++) ; // TODO: -- using timer!
  (*(instance->DATA_Drive))(0); // 2)   Bring the Data line low 
  (*(instance->SCLK_Drive))(1); // 3)   Release the Clock line.
  while ((*(instance->SCLK_Get))() != 0) ; // 4)   Wait for the device to bring the Clock line low. 
  // data bit
  for (i = 0; i < 8; i++)
  {
    if (data & (1 << i)) //  5)   Set/reset the Data line to send the first data bit 
    {
      (*(instance->DATA_Drive))(1);
    }
    else
    {
      (*(instance->DATA_Drive))(0);
    }
    while ((*(instance->SCLK_Get))() == 0) ; // 6)   Wait for the device to bring Clock high. 
    while ((*(instance->SCLK_Get))() != 0) ; // 7)   Wait for the device to bring Clock low. 
  }
  // parity bit
  if (compute_parity(data)) //  5)   Set/reset the Data line to send the first data bit 
  {
    (*(instance->DATA_Drive))(1);
  }
  else
  {
    (*(instance->DATA_Drive))(0);
  }
  while ((*(instance->SCLK_Get))() == 0) ; // 6)   Wait for the device to bring Clock high. 
  while ((*(instance->SCLK_Get))() != 0) ; // 7)   Wait for the device to bring Clock low. 
  
  // 8)   Repeat steps 5-7 for the other seven data bits and the parity bit
  
  (*(instance->DATA_Drive))(1); // 9)   Release the Data line. 
  while ((*(instance->DATA_Get))() != 0) ; // 10) Wait for the device to bring Data low. 
  while ((*(instance->SCLK_Get))() != 0) ; // 11) Wait for the device to bring Clock low. 
  
  while ((*(instance->DATA_Get))() == 0) ; // 12) Wait for the device to release Data and Clock  
  while ((*(instance->SCLK_Get))() == 0) ;
  
  instance->TimeOut = 1;
}
