/* University of York Robotics Laboratory PsiSwarm Library: Eprom Functions Header File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: eprom.h
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
 */


#ifndef EPROM_H
#define EPROM_H


//FIRMWARE DEFAULTS
//Batch number (0-255), Serial number (1 - 255)
#define FD_BATCH_NUMBER 3
#define FD_SERIAL_NUMBER 1
#define FD_CREATE_DAY 01
#define FD_CREATE_MONTH 06
#define FD_CREATE_YEAR 17
#define FD_PCB_VERSION_BIG 1
#define FD_PCB_VERSION_LITTLE 6

//Firmware Version (eg 1.1)
#define FD_FIRMWARE_VERSION_BIG 1
#define FD_FIRMWARE_VERSION_LITTLE 2
#define FD_HAS_COMPASS 0
#define FD_HAS_SIDE_IR 1
#define FD_HAS_BASE_IR 1
#define FD_HAS_BASE_COLOUR 1
#define FD_HAS_TOP_COLOUR 0
#define FD_HAS_ENCODERS 0
#define FD_HAS_AUDIO_PIC 0
#define FD_HAS_ULTRASONIC 0
#define FD_HAS_TEMPERATURE 1
#define FD_HAS_RECHARGING 1
#define FD_HAS_433_RADIO 0


/** Eprom Class
 * Functions for accessing the 64Kb EPROM chip and reading the reserved firmware block
 *
 * Example:
 * @code
 * #include "psiswarm.h"
 *
 * int main() {
 *     init();
 *     eprom.write_eeprom_byte(0,0xDD);    //Writes byte 0xDD in EPROM address 0
 *     char c = eprom.read_eeprom_byte(0); //c will hold 0xDD
 *     //Valid address range is from 0 to 65279
 * }
 * @endcode
 */
class Eprom
{

public:

    /** Write a single byte to the EPROM
     *
     * @param address The address to store the data, range 0-65279
     * @param data The character to store
     */
    void write_eeprom_byte ( int address, char data );

    /** Read a single byte from the EPROM
     *
     * @param address The address to read from, range 0-65279
     * @return The character stored at address
     */
    char read_eeprom_byte ( int address );

    /** Read the next byte from the EPROM, to be called after read_eeprom_byte
     *
     * @return The character stored at address after the previous one read from
     */
    char read_next_eeprom_byte ( void );

    /** Read the data stored in the reserved firmware area of the EPROM
     *
     * @return 1 if a valid firmware is read, 0 otherwise
     */
    char read_firmware ( void );
    
    /** Update the firmware to include features from the current software version
     *
     */
    void update_firmware( void );
    
    void write_firmware(char _flash_count, char _create_day, char _create_month, char _create_year, char _batch_number, char _serial_number, char _pcb_version_big, char _pcb_version_little, char _firmware_version_big, 
                           char _firmware_version_little, char _has_compass, char _has_side_ir, char _has_base_ir, char _has_base_colour_sensor, char _has_top_colour_sensor, char _has_encoders, char _has_audio_pic, char _has_ultrasonic,
                           char _has_temperature, char _has_recharging, char _has_433_radio, char _motor_calibration_set, char _base_ir_calibration_set, char _base_colour_calibration_set, short _boot_count);


    void write_string(char value);
    
    void write_firmware_byte ( int address, char data );
    char read_firmware_byte ( int address );
    
    void firmware_writer(void);
    const char * IF_get_state(char value);
    void IF_write_base_ir_calibration_values(short white_values[5], short black_values[5]);
    void IF_write_base_colour_calibration_values(int white_values[4], int black_values[4]);
    void IF_write_motor_calibration_values(float left_motor_calibration_value, int left_motor_offset, float right_motor_calibration_value, int right_motor_offset);
};

#endif