/* University of York Robotics Laboratory PsiSwarm Library: PsiSwarm Settings File
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: settings.h
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
 *
 */ 

/*! \file settings.h
    \brief Header file containing PsiSwarm define headings
    \brief USE_MOTOR_CALIBRATION Enable motor calibration using stored values. 
    If enabled, the actual motor speeds will be adjusted from the requested values using the motor calibration values stored in EPROM.
    \brief OFFSET_MOTORS Enable PWM offset to prevent stalling at low speeds. The motors typically stall when the PWM output is below around 0.2. 
    Enabling the offset shifts the actual PWM range so that the motor speed 0.0 -> 1.0 actually sets a PWM output of 0.2 -> 1.0
    
*/

#ifndef SETTINGS_H
#define SETTINGS_H

/* USE_MOTOR_CALIBRATION [1=on, recommended      0=off]
 * If enabled, the actual motor speeds will be adjusted from the requested values using the motor calibration values stored in
 * firmware (and set using the motor calibration code in the demo mode).  If calibration values not stall, will still offset
 * motors if OFFSET_MOTORS is enable.
 */
#define USE_MOTOR_CALIBRATION 1

/* OFFSET_MOTORS [1=on, recommended      0=off]
 * The motors typically stall when the PWM output is below around 0.2
 * Enabling the offset shifts the actual PWM range so that the motor speed 0.0 -> 1.0 actually sets a PWM output of 0.2 -> 1.0
 * If USE_MOTOR_CALIBRATION enable and calibration values for robot stored, OFFSET_MOTORS is ignored as offset is included in calibration
 */
#define OFFSET_MOTORS 1 

/* ENABLE_DEMO [1=on, 0=off]
 * If enabled the demo mode will be launched when the center button is held as the MBED is switched on or reset
 */
#define ENABLE_DEMO 1


/* ENABLE_BASIC [1=on, 0=off]:  Enable if the Psi-BASIC Interpretter is being used */
#define ENABLE_BASIC 1

/* SERIAL INTERFACES SETTINGS 
 * __________________________
 *
 * The Psi-Swarm can communicate using both the USB PC-Serial interface or via Bluetooth using a BlueSMIRF module
 *
 *
 */

/* ENABLE_BLUETOOTH [1=on, 0=off]:  Enable if the BlueSmirf module is being used */
/** @brief Enable if the BlueSmirf module is being used. 0=off 1=on*/
#define ENABLE_BLUETOOTH 1

/* ENABLE_PC_SERIAL [1=on, 0=off]:  Enable if the PC USB serial module is being used */
#define ENABLE_PC_SERIAL 1

/* BLUETOOTH_BAUD [recommended=115200]:  Baud rate for the BlueSMIRF module */
#define BLUETOOTH_BAUD 115200

/* PC_BAUD [recommended=460800 for optimal performance, 115200 for compatability]:  Baud rate for the PC USB serial module */
//#define PC_BAUD 460800
#define PC_BAUD 9600

/* DEBUG_MODE [1=on, 0=off]:  Enable to allow debug messages to be sent of one of the serial interfaces */
#define DEBUG_MODE 1

/* SHOW_VR_WARNINGS [1=on, 0=off]:   Show voltage-regulator debug message warnings after initial boot-up */
#define SHOW_VR_WARNINGS 0

/* USE_LED3_FOR_INTERRUPTS [1=on, 0=off]:  Switch interrupts will appear as the lighting of LED3 on MBED */
#define USE_LED3_FOR_INTERRUPTS 1

/* USE_LED4_FOR_VR_WARNINGS [1=on, 0=off]:  Voltage-regulator warnings will appear as the lighting of LED4 on MBED */
#define USE_LED4_FOR_VR_WARNINGS 1

/* HALT_ON_GPIO_ERROR [1=on, 0=off]:  Halts system if no GPIO response received during init() - typically this happens when MBED is powered but robot is switched off */
#define HALT_ON_GPIO_ERROR 1

/* HALT_ON_ALL_VREGS_LOW [1=on, 0=off]:  Halts system when all 3.3V voltage regulators are low on boot-up - typically this happens when MBED is powered but robot is switched off */
#define HALT_ON_ALL_VREGS_LOW 0

/* DEBUG_OUTPUT_STREAM [1=PC\USB 2=BlueSmirf 4=Display]:  Specify which output stream(s) should be used by default for debug messages, if enabled*/
#define DEBUG_OUTPUT_STREAM 1

/* SELF_TEST_PERIOD [recommended = 20000]:  Step time (in uS) for the quick self-test in the demo mode*/
#define SELF_TEST_PERIOD 20000

/* SAMPLE_SIZE [recommended = 10]:  Number of samples to take for the quick self-test in the demo mode*/
#define SAMPLE_SIZE 10

/* BASE_COLOUR_SENSOR_INTEGRATION_TIME [recommended = 3]:  Integration time in mS, shortest is ~2.4mS [3]; see TCS34725 datasheet */ 
#define BASE_COLOUR_SENSOR_INTEGRATION_TIME 3

/* BASE_COLOUR_SENSOR_GAIN [recommended = 4]: Gain for colour sensor, can be 1,4,16 or 60 */
#define BASE_COLOUR_SENSOR_GAIN 4

/* ENABLE_FIRMWARE_WRITER [1=on,0=off]:  Will automatically enter firmware writer mode if no\invalid firmware detected */
#define ENABLE_FIRMWARE_WRITER 1

/* AUTO_UPDATE_FIRMWARE [1=on,0=off]:  If an old version of firmware is detected, auto-update to the current version (preserving settings) */
#define AUTO_UPDATE_FIRMWARE 1


#endif

