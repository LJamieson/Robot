/* University of York Robotics Laboratory PsiSwarm Library: PsiSwarm C++ Core Header File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: psiswarm.h
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

#ifndef PSISWARM_H
#define PSISWARM_H

#define SOFTWARE_VERSION_CODE 0.90
#define TARGET_FIRMWARE_VERSION 1.2

#define PIC_ADDRESS 0x30
#define LCD_ADDRESS 0x7C
#define LED_IC_ADDRESS 0x42
#define GPIO_IC_ADDRESS 0x40
#define AUX_IC_ADDRESS 0x4E
#define ADC1_ADDRESS 0x46
#define ADC2_ADDRESS 0x48
#define EEPROM_ADDRESS 0XA0
#define TEMPERATURE_ADDRESS 0X30
#define BASE_COLOUR_ADDRESS 0X52
#define TOP_COLOUR_ADDRESS 0X72
#define ULTRASONIC_ADDRESS 0XE0
#define COMMAND_MESSAGE_BYTE 0X1D
#define ACKNOWLEDGE_MESSAGE_BYTE 0X1E
#define RESPONSE_MESSAGE_BYTE 0X1F
#define PSI_BYTE 0X1D

//Set temperature sensor warnings: 60C critical, 50C high, 0C low
#define TEMPERATURE_CRITICAL_HI 0X03
#define TEMPEARTURE_CRITICAL_LO 0XC0
#define TEMPERATURE_HIGH_HI 0X03
#define TEMPEARTURE_HIGH_LO 0X20
#define TEMPERATURE_LOW_HI 0X00
#define TEMPEARTURE_LOW_LO 0X00

/* CALIBRATION DEFAULTS 
 * __________________________
 *
 * These values are used as defaults if robot specific calibration values have not been stored in EEPROM
 *
 */
#define LEFT_STALL 20
#define RIGHT_STALL 20 
#define BIR1W 1750
#define BIR2W 3780
#define BIR3W 3820
#define BIR4W 3810
#define BIR5W 2880
#define BIR1B 30
#define BIR2B 350
#define BIR3B 670
#define BIR4B 550
#define BIR5B 250
#define CS_C_BLACK   63
#define CS_C_WHITE   802
#define CS_R_BLACK   22
#define CS_R_WHITE   244
#define CS_G_BLACK   24
#define CS_G_WHITE   297
#define CS_B_BLACK   16
#define CS_B_WHITE   232


#define DEFAULT_IR_PULSE_DELAY 400
#define DEFAULT_BASE_IR_PULSE_DELAY 50

#include <stdio.h>
#include <stdarg.h>
#include <string>
#include <vector>

#include "settings.h"
#include "serial.h"
#include "mbed.h"
#include "display.h"
#include "led.h"
#include "i2c_setup.h"
#include "motors.h"
#include "sensors.h"
#include "eprom.h"
#include "colour.h"
#include "sound.h"
#include "demo.h"
#include "animations.h"
#include "basic.h"

//NB The user needs to have a main.cpp with a main() function and a handle_switch_event(char) function
#include "main.h"

/** Psiswarm Class
 * The main class to define a robot
 *
 * Example code for main.cpp:
 * @code
 * #include "psiswarm.h"
 * Psiswarm psi;
 * char * program_name = "Example";
 * char * author_name  = "Name";
 * char * version_name = "0.9";
 * void handle_switch_event(char switch_state){}
 * void handle_user_serial_message(char * message, char length, char interface) {}
 * int main(){
 *    psi.init();
 *    while(1) { //Do something!
 *    }
 * }
 * @endcode
 */
class Psiswarm
{
public:
    /**
     * Main initialisation routine for the PsiSwarm robot
     *
     * Set up the GPIO expansion ICs, launch demo mode if button is held
     */
    void init(void);
    
    /**
     * Get the uptime for the robot
     *
     * @return The amount of time in seconds that the MBED has been active since last reset
     */
    float get_uptime(void);
    
    /**
     * Pause the user code for a defined amount of time
     *
     * @param The amount of time in seconds to pause user code
     */
    void pause_user_code(float period);
    
    /**
     * Reset the wheel encoder counters
     */
    void reset_encoders(void);
    
    /**
     * Send a string (in printf format) to the preferred debug stream, specified in settings.h [of overridden programmatically]
     *
     * @param The string to send to output stream
     */
    void debug(const char* format, ...) ;
private:
    void IF_end_pause_user_code(void);
    void IF_handle_events(void);
    void IF_update_encoders(void);
    void IF_update_user_id(void);
    void IF_update_switch(void);
    void IF_update_minutes(void);
    void IF_get_hardware_description(void);
};

extern "C" void mbed_reset();  // Allows use of (undocumented) mbed_reset() function   

extern Psiswarm psi;
extern Serial pc;
extern Serial bt;
extern Display display;
extern Motors motors;
extern Eprom eprom;
extern Led led;
extern Sensors sensors;
extern SerialControl serial;
extern Sound sound;
extern Setup i2c_setup;
extern Demo demo;
extern Animations animations;
extern Basic basic;
extern Colour colour;
extern char * program_name;
extern char * author_name;
extern char * version_name;

extern I2C primary_i2c;
extern InterruptIn gpio_interrupt;

extern AnalogIn vin_current;
extern AnalogIn vin_battery;
extern AnalogIn vin_dc;

extern DigitalOut mbed_led1;
extern DigitalOut mbed_led2;
extern DigitalOut mbed_led3;
extern DigitalOut mbed_led4;
extern PwmOut center_led_red;
extern PwmOut center_led_green;
extern PwmOut motor_left_f;
extern PwmOut motor_left_r;
extern PwmOut motor_right_f;
extern PwmOut motor_right_r;

extern char time_based_motor_action;

extern int base_colour_sensor_raw_values [];
extern int top_colour_sensor_raw_values [];

extern char waiting_for_ultrasonic;
extern int ultrasonic_distance;
extern char ultrasonic_distance_updated;
extern Timeout ultrasonic_timeout;
extern Ticker ultrasonic_ticker;

extern unsigned short background_ir_values [];
extern unsigned short illuminated_ir_values [];
extern float reflected_ir_distances [];
extern char ir_values_stored;

extern char firmware_bytes[];

extern char testing_voltage_regulators_flag;
extern char power_good_motor_left;
extern char power_good_motor_right;
extern char power_good_infrared;
extern char status_dc_in;
extern char status_charging;

extern unsigned short background_base_ir_values [];
extern unsigned short illuminated_base_ir_values [];
extern char base_ir_values_stored;
extern float line_position;
extern char line_found;

extern float motor_left_speed;
extern float motor_right_speed;
extern char motor_left_brake;
extern char motor_right_brake;

extern float center_led_brightness;
extern float backlight_brightness;

extern char use_motor_calibration;
extern char motor_calibration_set;
extern float left_motor_calibration_value;
extern float right_motor_calibration_value;
extern float left_motor_stall_offset;
extern float right_motor_stall_offset;

extern char base_ir_calibration_set;
extern char base_colour_calibration_set;

extern float firmware_version;
extern float pcb_version;
extern float serial_number;

extern char has_compass;
extern char has_side_ir;
extern char has_base_ir;
extern char has_base_colour_sensor;
extern char has_top_colour_sensor;
extern char has_wheel_encoders;
extern char has_audio_pic;
extern char has_ultrasonic_sensor;
extern char has_temperature_sensor;
extern char has_recharging_circuit;
extern char has_433_radio;

extern int ir_pulse_delay;
extern int base_ir_pulse_delay;

extern char robot_id;
extern char previous_robot_id;

extern char wheel_encoder_byte;
extern char previous_wheel_encoder_byte;
extern signed int left_encoder;
extern signed int right_encoder;

extern char switch_byte;
extern char previous_switch_byte;

extern char user_code_running;
extern char user_code_restore_mode;
extern char demo_on;
extern char event;
extern char change_id_event;
extern char encoder_event;
extern char switch_event;
extern char system_warnings;
extern short boot_count;

extern char debug_mode;
extern char debug_output;

extern vector<string> basic_filenames;
extern char psi_basic_file_count;
extern char use_flash_basic;
extern char file_transfer_mode;

#endif
