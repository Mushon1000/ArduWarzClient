#ifndef _VSARDUINO_H_
#define _VSARDUINO_H_
//Board = Digistump DigiX (standard)
#define __SAM3X8E__
#define USB_PID 0x078A
#define USB_VID 0x16D0
#define USBCON
#define ARDUINO 150
#define ARDUINO_MAIN
#define printf iprintf
#define __SAM__
#define __sam__
#define F_CPU 84000000L
#define __cplusplus
#define __inline__
#define __asm__(x)
#define __extension__
#define __ATTR_PURE__
#define __ATTR_CONST__
#define __inline__
#define __asm__ 
#define __volatile__

#define __ICCARM__
#define __ASM
#define __INLINE
#define __GNUC__ 0
#define __ICCARM__
#define __ARMCC_VERSION 400678
#define __attribute__(noinline)

#define prog_void
#define PGM_VOID_P int
            
typedef unsigned char byte;
extern "C" void __cxa_pure_virtual() {;}




void timeManager();
void PacketRecvManager();
//
//
void ledblinkRED(int Delay);
void ledblinkBLUE(int Delay);
void FlushInput();
unsigned long sendAck(unsigned long last_seq_num);
boolean calc_crc32(uint32_t crc, const char *buf, size_t len);
struct OperationPacket recvPacketNew();
void Setup_SW_Reset(int opcode, int dummy, int duration);
void Setup_Motors_Power(int opcode, int dummy, int duration);
void Motor_1_Forward();
void Motor_1_Stop();
void Motor_1_Backward();
void Motor_2_Forward();
void Motor_2_Stop();
void Motor_2_Backward();
void Servo_1_forward_steps(int opcode, int dummy, int duration);
void Servo_1_backward_steps(int opcode, int dummy, int duration);
void Servo_1_forward_step();
void Servo_1_backward_step();
void go_forward();
void go_forward_timed(int opcode, int dummy, int duration);
void go_reverse();
void go_reverse_timed(int opcode, int dummy, int duration);
void stop_go_forward(int opcode, int dummy, int duration);
void turn_left();
void go_left_timed(int opcode, int dummy, int duration);
void turn_right();
void go_right_timed(int opcode, int dummy, int duration);
void MessageHandlingUNITCOMMAND(int msgDataOffset);

#include "C:\Users\Shai\Documents\Arduino\hardware\digistump\sam\cores\digix\arduino.h"
#include "C:\Users\Shai\Documents\Arduino\hardware\digistump\sam\variants\digix\pins_arduino.h" 
#include "C:\Users\Shai\Documents\Arduino\hardware\digistump\sam\variants\digix\variant.h" 
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\UDPandEvent1_multi_chassis.ino"
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\EventDispatcher.cpp"
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\EventDispatcher.h"
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\EventQueue.cpp"
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\EventQueue.h"
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\Events.h"
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\ServoTimers.h"
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\interrupt.h"
#include "C:\Drive\Ideas\ArduinoWarz\Multi-platform chassis\UDPandEvent1_multi_chassis\wiring_constants.h"
#endif
