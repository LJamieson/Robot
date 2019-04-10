/* University of York Robotics Laboratory PsiSwarm Library: Serial Control Header File
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: serial.h
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
 
 
#ifndef SERIAL_H
#define SERIAL_H

/**
 * SerialControl class
 * Functions to handle command and user messages sent over the PC or BT serial interfaces.  Most of the functions within this class
 * are not intended to be called by user applications; once the setup_serial_interfaces() function has been called the enabled 
 * serial ports are attached to listeners which handle any received messages.  A predefined message structure for commands has been
 * created which allows most functions on the robot to be externally called, either using a PC-robot or Bluetooth connection.
 *
 * For user functions, the main.cpp file should include a void handle_user_serial_message(char * message, char length, char interface)
 * function to handle user-defined messages.
 *
*/
class SerialControl
{
public:
//void handle_user_serial_message(char * message, char length, char interface);

/**
 *  Sets the baud rates and enables the serial interfaces (PC and BT) as defined in the settings.h file
 *  Attaches listeners to both the serial ports that trigger when a message is received
 */
void setup_serial_interfaces(void);

private:
void IF_start_file_transfer_mode(void);
void IF_end_file_transfer_mode(void);
void IF_file_transfer_timeout(void);
void IF_handle_file_transfer_serial_message(char * message, char length, char interface);
void IF_handle_user_serial_message(char * message, char length, char interface);
void IF_handle_command_serial_message(char message [3], char interface);
void IF_invalid_transfer(void);
void IF_pc_rx_callback(void);
void IF_bt_rx_callback(void);
void IF_pc_rx_command_timeout(void);
void IF_bt_rx_command_timeout(void);
void IF_bt_message_timeout(void);
char * IF_nibble_to_binary_char(char in);
char * IF_char_to_binary_char(char in);
float IF_decode_unsigned_float(char byte0, char byte1);
float IF_decode_float(char byte0, char byte1);
float IF_decode_float(char byte0);
float IF_decode_unsigned_float(char byte0);
void IF_set_filename(char * filename_in);
unsigned short IF_calculateCRC16(int file_length);
};

#endif