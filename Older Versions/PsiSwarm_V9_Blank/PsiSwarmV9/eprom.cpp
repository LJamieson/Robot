/** University of York Robotics Laboratory PsiSwarm Library: Eprom Functions Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: eprom.cpp
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
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

#include "psiswarm.h"

/** Write a single byte to the EPROM
 *
 * @param address The address to store the data, range 0-65279
 * @param data The character to store
 */
void Eprom::write_eeprom_byte ( int address, char data )
{
    char write_array[3];
    if(address > 65279) {
        psi.debug("WARNING: Attempt to write to invalid EPROM address: %X",address);
    } else {
        write_array[0] = address / 256;
        write_array[1] = address % 256;
        write_array[2] = data;
        primary_i2c.write(EEPROM_ADDRESS, write_array, 3, false);
        //Takes 5ms to write a page: ideally this could be done with a timer or RTOS
        wait(0.005);
    }
}

/** Read a single byte from the EPROM
 *
 * @param address The address to read from, range 0-65279
 * @return The character stored at address
 */
char Eprom::read_eeprom_byte ( int address )
{
    char address_array [2];
    address_array[0] = address / 256;
    address_array[1] = address % 256;
    char data [1];
    primary_i2c.write(EEPROM_ADDRESS, address_array, 2, false);
    primary_i2c.read(EEPROM_ADDRESS, data, 1, false);
    return data [0];
}

/** Read the next byte from the EPROM, to be called after read_eeprom_byte
 *
 * @return The character stored at address after the previous one read from
 */
char Eprom::read_next_eeprom_byte ()
{
    char data [1];
    primary_i2c.read(EEPROM_ADDRESS, data, 1, false);
    return data [0];
}

void Eprom::write_firmware_byte ( int address, char data ){
    char write_array[3];
    write_array[0] = 255;
    write_array[1] = address % 256;
    write_array[2] = data;
    primary_i2c.write(EEPROM_ADDRESS, write_array, 3, false);
    //Takes 5ms to write a page: ideally this could be done with a timer or RTOS
    wait(0.005);
}
  
    
char Eprom::read_firmware_byte ( int address ){
    char address_array [2];
    address_array[0] = 255;
    address_array[1] = address % 256;
    char data [1];
    primary_i2c.write(EEPROM_ADDRESS, address_array, 2, false);
    primary_i2c.read(EEPROM_ADDRESS, data, 1, false);
    return data [0];
}

/** Read the data stored in the reserved firmware area of the EPROM
 *
 * @return 1 if a valid firmware is read, 0 otherwise
 */
char Eprom::read_firmware ()
{
    char address_array [2] = {255,0};
    primary_i2c.write(EEPROM_ADDRESS, address_array, 2, false);
    primary_i2c.read(EEPROM_ADDRESS, firmware_bytes, 80, false);
    left_motor_stall_offset = LEFT_STALL * 0.01f;
    right_motor_stall_offset = RIGHT_STALL * 0.01f;
    if(firmware_bytes[0] == PSI_BYTE) {
        // Parse firmware
        char firmware_string [8];
        sprintf(firmware_string,"%d.%d",firmware_bytes[9],firmware_bytes[10]);
        firmware_version = atof(firmware_string);
        char pcb_version_string [8];
        sprintf(pcb_version_string,"%d.%d",firmware_bytes[7],firmware_bytes[8]);
        pcb_version = atof(pcb_version_string);
        char serial_number_string [8];
        if(firmware_bytes[6] > 9) sprintf(serial_number_string,"%d.%d",firmware_bytes[5],firmware_bytes[6]);
        else sprintf(serial_number_string,"%d.0%d",firmware_bytes[5],firmware_bytes[6]);
        serial_number = atof(serial_number_string);
        has_compass = firmware_bytes[11];
        has_side_ir = firmware_bytes[12];
        has_base_ir = firmware_bytes[13];
        has_base_colour_sensor= firmware_bytes[14];
        has_top_colour_sensor= firmware_bytes[15];
        has_wheel_encoders= firmware_bytes[16];
        has_audio_pic= firmware_bytes[17];
        has_ultrasonic_sensor= firmware_bytes[18];
        has_temperature_sensor= firmware_bytes[19];
        has_recharging_circuit= firmware_bytes[20];
        has_433_radio= firmware_bytes[21];
        if(firmware_version > 1.0) {
            motor_calibration_set = firmware_bytes[22];
            if(motor_calibration_set == 1) {
                left_motor_calibration_value = (float) firmware_bytes[23] * 65536;
                left_motor_calibration_value += ((float) firmware_bytes[24] * 256);
                left_motor_calibration_value += firmware_bytes[25];
                left_motor_calibration_value /= 16777216;
                right_motor_calibration_value = (float) firmware_bytes[26] * 65536;
                right_motor_calibration_value += ((float) firmware_bytes[27] * 256);
                right_motor_calibration_value += firmware_bytes[28];
                right_motor_calibration_value /= 16777216;
            } else motor_calibration_set = 0;
        } else motor_calibration_set = 0;
        if(firmware_version > 1.1) {
            boot_count = firmware_bytes[69] << 8;
            boot_count += firmware_bytes[70];
            boot_count++;
            eprom.write_firmware_byte(69,(char)(boot_count >> 8));
            eprom.write_firmware_byte(70,(char)(boot_count % 256));
            if(motor_calibration_set == 1) {
                left_motor_stall_offset = (((float) (firmware_bytes[67])) * 0.01f);
                right_motor_stall_offset = (((float) (firmware_bytes[68])) * 0.01f);
            }
            base_ir_calibration_set = firmware_bytes[29];
            if(base_ir_calibration_set == 1){
              int white_values[5];
              int black_values[5];
              for(int i=0;i<5;i++){
                  int k_val = i+i;
                  white_values[i] = (firmware_bytes[30 + k_val] << 8) + firmware_bytes[31 + k_val];
                  black_values[i] = (firmware_bytes[40 + k_val] << 8) + firmware_bytes[41 + k_val];  
              }
              sensors.IF_set_base_calibration_values(white_values[0], white_values[1], white_values[2], white_values[3], white_values[4], black_values[0], black_values[1], black_values[2], black_values[3], black_values[4]);
            }else base_ir_calibration_set = 0;
            base_colour_calibration_set = firmware_bytes[50];
            if(base_colour_calibration_set == 1){
                int white_values[4];
                int black_values[4];
                for(int i=0;i<4;i++){
                   int k_val = i+i;
                  white_values[i] = (firmware_bytes[51 + k_val] << 8) + firmware_bytes[52 + k_val];
                  black_values[i] = (firmware_bytes[59 + k_val] << 8) + firmware_bytes[60 + k_val];  
                }
                colour.set_calibration_values(black_values[0],black_values[1],black_values[2],black_values[3],white_values[0],white_values[1],white_values[2],white_values[3]);
            }else base_colour_calibration_set = 0;
        } else {
            base_ir_calibration_set = 0;
            base_colour_calibration_set = 0;
        }
        return 1;
    }
    return 0;
}


void Eprom::firmware_writer()
{
    psi.debug("Starting firmware writer function\n");
    display.clear_display();
    display.write_string("FIRMWARE WRITER");
    display.set_position(1,0);
    int sub_menu = 0;
    char _batch_number = FD_BATCH_NUMBER;
    char _serial_number = FD_SERIAL_NUMBER;
    char _create_day = FD_CREATE_DAY;
    char _create_month = FD_CREATE_MONTH;
    char _create_year = FD_CREATE_YEAR;
    char _pcb_version_big = FD_PCB_VERSION_BIG;
    char _pcb_version_little = FD_PCB_VERSION_LITTLE;
    char _firmware_version_big = FD_FIRMWARE_VERSION_BIG;
    char _firmware_version_little = FD_FIRMWARE_VERSION_LITTLE;
    char _has_compass = FD_HAS_COMPASS;
    char _has_side_ir = FD_HAS_SIDE_IR;
    char _has_base_ir = FD_HAS_BASE_IR;
    char _has_base_colour = FD_HAS_BASE_COLOUR;
    char _has_top_colour = FD_HAS_TOP_COLOUR;
    char _has_encoders = FD_HAS_ENCODERS;
    char _has_audio_pic = FD_HAS_AUDIO_PIC;
    char _has_ultrasonic = FD_HAS_ULTRASONIC;
    char _has_temperature = FD_HAS_TEMPERATURE;
    char _has_recharging = FD_HAS_RECHARGING;
    char _has_433_radio = FD_HAS_433_RADIO;
    char exit_state = 0;
    char old_button = 255;
    char button;
    while(exit_state == 0){
        button = i2c_setup.IF_get_switch_state();
        if(button != old_button){
            old_button = button;
            if(button == 1)// Up
            {
                switch(sub_menu){
                    case 0: if(_batch_number == 0) _batch_number = 20;
                    else _batch_number --;
                    break;
                    case 1: if(_serial_number == 0) _serial_number = 99;
                    else _serial_number --;
                    break;
                    case 2: if(_create_day == 1) _create_day = 31;
                    else _create_day --;
                    break;
                    case 3: if(_create_month == 1) _create_month = 12;
                    else _create_month --;
                    break;
                    case 4: if(_create_year == 15) _create_year = 25;
                    else _create_year --;
                    break;
                    case 5: if(_pcb_version_big == 1) _pcb_version_big = 3;
                    else _pcb_version_big --;
                    break;
                    case 6: if(_pcb_version_little == 0) _pcb_version_little = 9;
                    else _pcb_version_little --;
                    break;
                    case 7: if(_firmware_version_big == 1) _firmware_version_big = 3;
                    else _firmware_version_big --;
                    break;
                    case 8: if(_firmware_version_little == 0) _firmware_version_little = 9;
                    else _firmware_version_little --;
                    break;
                    case 9: _has_compass = 1-_has_compass; break;
                    case 10: _has_side_ir = 1 - _has_side_ir; break;
                    case 11: _has_base_ir = 1 - _has_base_ir; break;
                    case 12: _has_base_colour = 1 - _has_base_colour; break;
                    case 13: _has_top_colour = 1 - _has_top_colour; break;
                    case 14: _has_encoders = 1 - _has_encoders; break;
                    case 15: _has_audio_pic = 1 - _has_audio_pic; break;
                    case 16: _has_ultrasonic = 1 - _has_ultrasonic; break;
                    case 17: _has_temperature = 1 - _has_temperature; break;
                    case 18: _has_recharging = 1 - _has_recharging; break;
                    case 19: _has_433_radio = 1 - _has_433_radio; break;
                    case 20: exit_state = 1; break;
                    case 21: exit_state = 2; break;
                }
            }
            if(button == 2)// Down
            {
                switch(sub_menu){
                    case 0: if(_batch_number == 20) _batch_number = 0;
                    else _batch_number ++;
                    break;
                    case 1: if(_serial_number == 99) _serial_number = 0;
                    else _serial_number ++;
                    break;
                    case 2: if(_create_day == 31) _create_day = 1;
                    else _create_day ++;
                    break;
                    case 3: if(_create_month == 12) _create_month = 1;
                    else _create_month ++;
                    break;
                    case 4: if(_create_year == 25) _create_year = 15;
                    else _create_year ++;
                    break;
                    case 5: if(_pcb_version_big == 3) _pcb_version_big = 1;
                    else _pcb_version_big ++;
                    break;
                    case 6: if(_pcb_version_little == 9) _pcb_version_little = 0;
                    else _pcb_version_little ++;
                    break;
                    case 7: if(_firmware_version_big == 3) _firmware_version_big = 1;
                    else _firmware_version_big ++;
                    break;
                    case 8: if(_firmware_version_little == 9) _firmware_version_little = 0;
                    else _firmware_version_little ++;
                    break;
                    case 9: _has_compass = 1-_has_compass; break;
                    case 10: _has_side_ir = 1 - _has_side_ir; break;
                    case 11: _has_base_ir = 1 - _has_base_ir; break;
                    case 12: _has_base_colour = 1 - _has_base_colour; break;
                    case 13: _has_top_colour = 1 - _has_top_colour; break;
                    case 14: _has_encoders = 1 - _has_encoders; break;
                    case 15: _has_audio_pic = 1 - _has_audio_pic; break;
                    case 16: _has_ultrasonic = 1 - _has_ultrasonic; break;
                    case 17: _has_temperature = 1 - _has_temperature; break;
                    case 18: _has_recharging = 1 - _has_recharging; break;
                    case 19: _has_433_radio = 1 - _has_433_radio; break;
                    case 20: exit_state = 1; break;
                    case 21: exit_state = 2; break;
                }
            }
            if(button == 4)// Left
            {
                if(sub_menu == 0) sub_menu = 21;
                else sub_menu --;
                
            }
            if(button == 8)// Right
            {
                if(sub_menu == 21) sub_menu = 0;
                else sub_menu ++;
            }
            display.set_position(1,0);
            char message[17];
            switch(sub_menu){
                case 0: sprintf(message,"BATCH NUMBER:%3d",_batch_number); break;
                case 1: sprintf(message,"SERIAL NUMBR:%3d",_serial_number); break;
                case 2: sprintf(message,"CREATE DAY  :%3d",_create_day); break;
                case 3: sprintf(message,"CREATE MONTH:%3d",_create_month); break;
                case 4: sprintf(message,"CREATE YEAR :%3d",_create_year); break;
                case 5: sprintf(message,"PCB VERS BIG:%3d",_pcb_version_big); break;
                case 6: sprintf(message,"PCB VERS LIT:%3d",_pcb_version_little); break;
                case 7: sprintf(message,"FWR VERS BIG:%3d",_firmware_version_big);break;
                case 8: sprintf(message,"FWR VERS LIT:%3d",_firmware_version_little); break;
                case 9: sprintf(message,"HAS COMPASS :%s",IF_get_state(_has_compass)); break;   
                case 10: sprintf(message,"HAS SIDE IR :%s",IF_get_state(_has_side_ir)); break;   
                case 11: sprintf(message,"HAS BASE IR :%s",IF_get_state(_has_base_ir)); break;   
                case 12: sprintf(message,"HAS BASE COL:%s",IF_get_state(_has_base_colour)); break;   
                case 13: sprintf(message,"HAS TOP COL :%s",IF_get_state(_has_top_colour)); break;   
                case 14: sprintf(message,"HAS ENCODERS:%s",IF_get_state(_has_encoders)); break;   
                case 15: sprintf(message,"HAS AUDIOPIC:%s",IF_get_state(_has_audio_pic)); break;   
                case 16: sprintf(message,"HAS ULTRASNC:%s",IF_get_state(_has_ultrasonic)); break;   
                case 17: sprintf(message,"HAS TEMPERAT:%s",IF_get_state(_has_temperature)); break;  
                case 18: sprintf(message,"HAS RECHARGE:%s",IF_get_state(_has_recharging)); break;   
                case 19: sprintf(message,"HAS 433RADIO:%s",IF_get_state(_has_433_radio)); break;   
                case 20: sprintf(message,"ACCEPT          "); break;   
                case 21: sprintf(message,"REJECT          "); break;   
            }
            display.write_string(message);
        }
        wait(0.01);
    }
    if(exit_state == 1){
         display.clear_display();
         display.write_string("WRITING FIRMWARE");
         char _flash_count = read_firmware_byte(1);
         if(_flash_count > 199) _flash_count = 0;
         char _motor_calibration_set = read_firmware_byte(22);
         if(_motor_calibration_set > 1) _motor_calibration_set = 0;
         char _base_ir_calibration_set = read_firmware_byte(29);
         if(_base_ir_calibration_set > 1) _base_ir_calibration_set = 0;
         char _base_colour_calibration_set = read_firmware_byte(50);
         if(_base_colour_calibration_set > 1) _base_colour_calibration_set = 0;
         int _boot_count = ((int)read_firmware_byte(69) << 8) + read_firmware_byte(70);
         if(_boot_count > 29999) _boot_count = 0;
         write_firmware(_flash_count, _create_day, _create_month, _create_year, _batch_number, _serial_number, _pcb_version_big, _pcb_version_little,  _firmware_version_big, 
                           _firmware_version_little, _has_compass, _has_side_ir, _has_base_ir, _has_base_colour, _has_top_colour, _has_encoders, _has_audio_pic, _has_ultrasonic,
                           _has_temperature, _has_recharging, _has_433_radio, _motor_calibration_set, _base_ir_calibration_set, _base_colour_calibration_set, _boot_count);
    } else {
         display.clear_display();
         display.write_string("CHANGES REJECTED");
         wait(1);
         psi.debug("Firmware writer settings rejected\n");
         display.clear_display();
    }
    wait(1);
}

const char * Eprom::IF_get_state(char value)
{
    if(value==1)return"YES";
    return"NO ";   
}

void Eprom::IF_write_base_ir_calibration_values(short white_values[5], short black_values[5]){
    //Set calibration_set byte [29] to 1
    write_firmware_byte(29,1);
    for(int i=0;i<5;i++){
       write_firmware_byte(30+i+i,(char) (white_values[i] >> 8));
       write_firmware_byte(31+i+i,(char) (white_values[i] % 256));
    }
     for(int i=0;i<5;i++){
       write_firmware_byte(40+i+i,(char) (black_values[i] >> 8));
       write_firmware_byte(41+i+i,(char) (black_values[i] % 256));
    }
    sensors.IF_set_base_calibration_values(white_values[0], white_values[1], white_values[2], white_values[3], white_values[4], black_values[0], black_values[1], black_values[2], black_values[3], black_values[4]);
}

void Eprom::IF_write_base_colour_calibration_values(int white_values[4], int black_values[4]){
    //Set calibration_set byte [50] to 1
    write_firmware_byte(50,1);
    for(int i=0;i<4;i++){
       write_firmware_byte(51+i+i,(char) (black_values[i] >> 8));
       write_firmware_byte(52+i+i,(char) (black_values[i] % 256));
    }
     for(int i=0;i<4;i++){
       write_firmware_byte(59+i+i,(char) (white_values[i] >> 8));
       write_firmware_byte(60+i+i,(char) (white_values[i] % 256));
    }
    colour.set_calibration_values(black_values[0],black_values[1],black_values[2],black_values[3],white_values[0],white_values[1],white_values[2],white_values[3]);
}

void Eprom::IF_write_motor_calibration_values(float left_motor_calibration_value, int left_motor_offset, float right_motor_calibration_value, int right_motor_offset){
    //Set calibration_set byte [22] to 1
    write_firmware_byte(22,1);
    int left_motor_cv = left_motor_calibration_value * 16777215;
    int right_motor_cv = right_motor_calibration_value * 16777215;
    char lm1 = (char)(left_motor_cv >> 16);
    char lm2 = (char)(left_motor_cv >> 8 % 256);
    char lm3 = (char)(left_motor_cv % 256);
    char rm1 = (char)(right_motor_cv >> 16);
    char rm2 = (char)(right_motor_cv >> 8 % 256);
    char rm3 = (char)(right_motor_cv % 256);
    write_firmware_byte(23,lm1);
    write_firmware_byte(24,lm2);
    write_firmware_byte(25,lm3);
    write_firmware_byte(26,rm1);
    write_firmware_byte(27,rm2);
    write_firmware_byte(28,rm3);   
    write_firmware_byte(67,left_motor_offset);
    write_firmware_byte(68,right_motor_offset);
}

void Eprom::update_firmware(){
    psi.debug("\n\nPsiSwarm Firmware Writer\n___________________________________\nUpdating firmware to version %1.1f\n",TARGET_FIRMWARE_VERSION);
    display.clear_display();
    display.set_position(0,0);
    display.write_string("UPDATING");
    display.set_position(1,0);
    display.write_string("FIRMWARE");
    char fv_big = (char) TARGET_FIRMWARE_VERSION;
    char fv_small = (char) ((float) (TARGET_FIRMWARE_VERSION - fv_big) * 10.0f);
  
    wait(0.5);
    eprom.write_firmware(firmware_bytes[1], FD_CREATE_DAY, FD_CREATE_MONTH, FD_CREATE_YEAR, firmware_bytes[5], firmware_bytes[6], firmware_bytes[7], firmware_bytes[8],fv_big, 
                         fv_small, has_compass, has_side_ir, has_base_ir, has_base_colour_sensor, has_top_colour_sensor, has_wheel_encoders, has_audio_pic, has_ultrasonic_sensor, has_temperature_sensor, has_recharging_circuit, 
                         has_433_radio, motor_calibration_set, base_ir_calibration_set, base_colour_calibration_set, boot_count);
}



void Eprom::write_string(char value){
    if(value==1) psi.debug("YES\n");
    else psi.debug("NO\n");   
}

void Eprom::write_firmware(char _flash_count, char _create_day, char _create_month, char _create_year, char _batch_number, char _serial_number, char _pcb_version_big, char _pcb_version_little, char _firmware_version_big, 
                           char _firmware_version_little, char _has_compass, char _has_side_ir, char _has_base_ir, char _has_base_colour_sensor, char _has_top_colour_sensor, char _has_encoders, char _has_audio_pic, char _has_ultrasonic,
                           char _has_temperature, char _has_recharging, char _has_433_radio, char _motor_calibration_set, char _base_ir_calibration_set, char _base_colour_calibration_set, short _boot_count){
    psi.debug("Writing new firmware:\n");
    write_firmware_byte(0,PSI_BYTE);
    _flash_count ++;
    psi.debug("Flash Count: %d\n",_flash_count);
    write_firmware_byte(1,_flash_count);
    psi.debug("Flash Date: %d-%d-%d\n",_create_day,_create_month,_create_year);
    write_firmware_byte(2,_create_day);
    write_firmware_byte(3,_create_month);
    write_firmware_byte(4,_create_year);
    psi.debug("Serial Number: %d-%d\n",_batch_number,_serial_number);
    write_firmware_byte(5,_batch_number);
    write_firmware_byte(6,_serial_number);
    psi.debug("PCB Version: %d.%d\n",_pcb_version_big,_pcb_version_little);
    write_firmware_byte(7,_pcb_version_big);
    write_firmware_byte(8,_pcb_version_little);
    psi.debug("Firmware Version: %d.%d\n",_firmware_version_big,_firmware_version_little);
    write_firmware_byte(9,_firmware_version_big);
    write_firmware_byte(10,_firmware_version_little);
    psi.debug("Has Compass: ");
    write_string(_has_compass);
    write_firmware_byte(11,_has_compass);
    psi.debug("Has Side IR Sensors: ");
    write_string(_has_side_ir);
    write_firmware_byte(12,_has_side_ir);
    psi.debug("Has Base IR Sensors: ");
    write_string(_has_base_ir);
    write_firmware_byte(13,_has_base_ir);
    psi.debug("Has Base Colour Sensor: ");
    write_string(_has_base_colour_sensor);
    write_firmware_byte(14,_has_base_colour_sensor);
    psi.debug("Has Top Colour Sensor: ");
    write_string(_has_top_colour_sensor);
    write_firmware_byte(15,_has_top_colour_sensor);
    psi.debug("Has Wheel Encoders: ");
    write_string(_has_encoders);
    write_firmware_byte(16,_has_encoders);
    psi.debug("Has Audio PIC: ");
    write_string(_has_audio_pic);
    write_firmware_byte(17,_has_audio_pic);
    psi.debug("Has Ultrasonic Sensor: ");
    write_string(_has_ultrasonic);
    write_firmware_byte(18,_has_ultrasonic);
    psi.debug("Has Temperature Sensor: ");
    write_string(_has_temperature);
    write_firmware_byte(19,_has_temperature);
    psi.debug("Has Recharging Circuit: ");
    write_string(_has_recharging);
    write_firmware_byte(20,_has_recharging);
    psi.debug("Has 433MHz Radio: ");
    write_string(_has_433_radio);
    write_firmware_byte(21,_has_433_radio);
    psi.debug("Motor calibration set: ");
    write_string(_motor_calibration_set);
    write_firmware_byte(22,_motor_calibration_set);
    psi.debug("Base IR calibration set: ");
    write_string(_base_ir_calibration_set);
    write_firmware_byte(29,_base_ir_calibration_set);
    psi.debug("Base colour calibration set: ");
    write_string(_base_colour_calibration_set);
    write_firmware_byte(50,_base_colour_calibration_set);
    psi.debug("Boot Count: %d\n",_boot_count);
    wait(0.2);
    psi.debug("_________________________________________\n");
    wait(0.2);
    psi.debug("VERIFYING FIRMWARE...\n");
    short test_b_c = read_firmware_byte(69) << 8;
    test_b_c += read_firmware_byte(70);
    if(read_firmware_byte(0) == PSI_BYTE
    && read_firmware_byte(1) == _flash_count
    && read_firmware_byte(2) == _create_day
    && read_firmware_byte(3) == _create_month
    && read_firmware_byte(4) == _create_year
    && read_firmware_byte(5) == _batch_number
    && read_firmware_byte(6) == _serial_number
    && read_firmware_byte(7) == _pcb_version_big
    && read_firmware_byte(8) == _pcb_version_little
    && read_firmware_byte(9) == _firmware_version_big
    && read_firmware_byte(10) == _firmware_version_little
    && read_firmware_byte(11) == _has_compass
    && read_firmware_byte(12) == _has_side_ir
    && read_firmware_byte(13) == _has_base_ir
    && read_firmware_byte(14) == _has_base_colour_sensor
    && read_firmware_byte(15) == _has_top_colour_sensor
    && read_firmware_byte(16) == _has_encoders
    && read_firmware_byte(17) == _has_audio_pic
    && read_firmware_byte(18) == _has_ultrasonic
    && read_firmware_byte(19) == _has_temperature
    && read_firmware_byte(20) == _has_recharging
    && read_firmware_byte(21) == _has_433_radio
    && read_firmware_byte(22) == _motor_calibration_set
    && read_firmware_byte(29) == _base_ir_calibration_set
    && read_firmware_byte(50) == _base_colour_calibration_set
    && test_b_c == _boot_count
    ){
        psi.debug("Flash successful.\n");
        display.clear_display();
    display.set_position(0,0);
    display.write_string("FIRMWARE");
    display.set_position(1,0);
    display.write_string("UPDATED");
    }
    else {psi.debug("ERROR: Corrupt data. Flashing failed.\n");
    display.clear_display();
    display.set_position(0,0);
    display.write_string("UPDATE");
    display.set_position(1,0);
    display.write_string("FAILED");
    wait(1);
    }
    
    wait(0.5);
    psi.debug("\nResetting...\n");
    wait(0.2);
    mbed_reset();    
}



//PCB Features
#define HAS_COMPASS 0
#define HAS_SIDE_IR 1
#define HAS_BASE_IR 1
#define HAS_ULTRASONIC 1
#define HAS_BASE_COLOUR_SENSOR 1
#define HAS_TOP_COLOUR_SENSOR 0
#define HAS_ENCODERS 0
#define HAS_AUDIO_PIC 0
#define HAS_TEMPERATURE 1
#define HAS_RECHARGING 1
#define HAS_433_RADIO 0

//Calibration
#define MOTOR_CALIBRATION_SET 0