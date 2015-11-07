/* 
	Editor: http://www.visualmicro.com
	        visual micro and the arduino ide ignore this code during compilation. this code is automatically maintained by visualmicro, manual changes to this file will be overwritten
	        the contents of the Visual Micro sketch sub folder can be deleted prior to publishing a project
	        all non-arduino files created by visual micro and all visual studio project or solution files can be freely deleted and are not required to compile a sketch (do not delete your own code!).
	        note: debugger breakpoints are stored in '.sln' or '.asln' files, knowledge of last uploaded breakpoints is stored in the upload.vmps.xml file. Both files are required to continue a previous debug session without needing to compile and upload again
	
	Hardware: Arduino/Genuino Mega w/ ATmega2560 (Mega 2560), Platform=avr, Package=arduino
*/

#define __AVR_ATmega2560__
#define ARDUINO 163
#define ARDUINO_MAIN
#define F_CPU 16000000L
#define __AVR__
#define F_CPU 16000000L
#define ARDUINO 163
#define ARDUINO_AVR_MEGA2560
#define ARDUINO_ARCH_AVR
extern "C" void __cxa_pure_virtual() {;}

//
//
void drawTFT();
void ReadDallas();
void InitializeOutputCard();
void pid1_WriteToOutput();
void pid2_WriteToOutput();
void pid1_changeAutoTune();
void pid2_changeAutoTune();
void pid1_AutoTuneHelper(boolean pid1_start);
void pid2_AutoTuneHelper(boolean pid2_start);
void pid1_StartProfile();
void pid2_StartProfile();
void pid1_StopProfile();
void pid2_StopProfile();
void pid1_ProfileRunTime();
void pid2_ProfileRunTime();
void pid1_calcNextProf();
void pid2_calcNextProf();
void initializeEEPROM();
void EEPROMreset();
void EEPROMBackupTunings();
void EEPROMRestoreTunings();
void EEPROMBackupATune();
void EEPROMRestoreATune();
void EEPROMBackupProfile();
void EEPROMRestoreProfile();
void SerialReceive();
void ReceiveSettings();
void ReceiveTunings();
void ReceiveAtune();
void pid1_ReceiveProfile();
void pid2_ReceiveProfile();
void ProfileCommand();
void SerialSend();

#include "C:\Users\sysop\AppData\Roaming\arduino15\packages\arduino\hardware\avr\1.6.9\variants\mega\pins_arduino.h" 
#include "C:\Users\sysop\AppData\Roaming\arduino15\packages\arduino\hardware\avr\1.6.9\cores\arduino\arduino.h"
#include <..\Dual_PID_Firmware\Dual_PID_Firmware.ino>
#include <..\Dual_PID_Firmware\EEPROMAnything.h>
#include <..\Dual_PID_Firmware\LCDI2Cw.cpp>
#include <..\Dual_PID_Firmware\LCDI2Cw_local.h>
#include <..\Dual_PID_Firmware\MAX31855.cpp>
#include <..\Dual_PID_Firmware\MAX31855_local.h>
#include <..\Dual_PID_Firmware\PID_AutoTune_v0.cpp>
#include <..\Dual_PID_Firmware\PID_AutoTune_v0_local.h>
#include <..\Dual_PID_Firmware\PID_v1.cpp>
#include <..\Dual_PID_Firmware\PID_v1_local.h>
#include <..\Dual_PID_Firmware\lcd_util.cpp>
#include <..\Dual_PID_Firmware\lcd_util.h>
