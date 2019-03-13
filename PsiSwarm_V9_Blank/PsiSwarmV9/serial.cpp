/* University of York Robotics Laboratory PsiSwarm Library: SerialControlControlControlControlControl Control Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: serial.cpp
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

#include "psiswarm.h"

static float command_timeout_period = 0.1f;     //If a complete command message is not received in 0.1s then consider it a user message
char pc_command_message_started = 0;
char pc_command_message_byte = 0;
char pc_command_message[3];
char bt_command_message_started = 0;
char bt_command_message_byte = 0;
char bt_command_message[3];

char allow_commands = 1;
char allow_requests = 1;
char file_transfer_state = 0;

int block_size = 88;   // The data block size for file transfer
char data_block[89];   // Stores the data block to write for Bluetooth file transfer
int data_written;       // Stores if partial data has been written to a file
int file_length;        // Stores the file length for a Bluetooth file transfer
int final_block;        // Stores the index of the final data block for a Bluetooth file transfer
int block_index;        // Stores the current block index for a Bluetooth file transfer
char filename [21];     // Stores the filename for a Bluetooth file transfer

Timeout ft_timeout;
Timeout pc_command_timeout;
Timeout bt_command_timeout;

// A predefined message structure for command messages is as follows:
// [Byte 0][Byte 1][Byte 2][Byte 3][Byte 4]
// Byte 0 and Byte 4 must be equal to COMMAND_MESSAGE_BYTE [in psiswarm.h] or message is treated as a user message


void SerialControl::setup_serial_interfaces()
{
    if(ENABLE_PC_SERIAL) {
        pc.baud(PC_BAUD);
        pc.attach(this,&SerialControl::IF_pc_rx_callback, Serial::RxIrq);
    }
    if(ENABLE_BLUETOOTH) {
        bt.baud(BLUETOOTH_BAUD);
        bt.attach(this,&SerialControl::IF_bt_rx_callback, Serial::RxIrq);
    }
}


void SerialControl::IF_start_file_transfer_mode()
{
    display.clear_display();
    display.set_position(0,0);
    display.write_string("FILE TRANSFER");
    display.set_position(1,0);
    display.write_string("MODE...");
    data_written = 0;
    file_transfer_mode = 1;
    file_transfer_state = 0;
    file_length = 0;
    user_code_restore_mode = user_code_running;
    user_code_running = 0;
    ft_timeout.attach(this,&SerialControl::IF_file_transfer_timeout,2.0);
}


void SerialControl::IF_invalid_transfer(void)
{
    psi.debug("File transfer failed\n");
    if(data_written == 1) {
        psi.debug("Deleting corrupted file\n");
        remove(filename);
    }
    display.clear_display();
    display.set_position(0,0);
    display.write_string("TRANSFER FAILED");
    wait(0.5);
    IF_end_file_transfer_mode();
}

void SerialControl::IF_file_transfer_timeout(void)
{
    psi.debug("File transfer failed: timeout\n");
    display.clear_display();
    display.set_position(0,0);
    display.write_string("TRANSFER TIMEOUT");
    wait(0.5);
    IF_end_file_transfer_mode();
}

void SerialControl::IF_end_file_transfer_mode(void)
{
    display.clear_display();
    file_transfer_mode = 0;
    user_code_running = user_code_restore_mode;
}


void SerialControl::IF_handle_file_transfer_serial_message(char * message, char length, char interface)
{
    // Code for handling a serial (Bluetooth) message when in file-transfer mode
    //
    // message = pointer to message char array
    // length = length of message
    // interface = 0 for PC serial connection, 1 for Bluetooth [NB only Bluetooth used for file transfer in this version]

    if(file_transfer_state < 2)psi.debug("FTM Message:%.*s [%d]\n",length,message,length);
    else psi.debug("FTM data block received (%d bytes)\n",length);
    int expected_size;
    // The first byte in EVERY message received should be 33; if it isn't, abort the transfer
    if(message[0] != 33) {
        IF_invalid_transfer();
    } else {
        switch(file_transfer_state) {
            case 0: //First message received is the target filename
                //The filenames cannot be more that 8.3 characters long (FAT12 format)
                if(length == 1 || length > 13) IF_invalid_transfer();
                else {
                    strcpy(filename, "/local/");
                    strncat(filename, message + 1, length - 1);
                    psi.debug("Target filename:%s\n",filename);
                    //Send acknowledge ("FN")
                    ft_timeout.detach();
                    ft_timeout.attach(this,&SerialControl::IF_file_transfer_timeout,2.0);
                    bt.printf("%c%c%s",RESPONSE_MESSAGE_BYTE,2,"FN");
                    file_transfer_state = 1;
                }
                break;
            case 1: //Second message is the length of the file in bytes
                //Length is encoded as a 3-byte value
                if(length != 4) IF_invalid_transfer();
                else {
                    file_length = (message[1]) * 256;
                    file_length += (message[2]);
                    file_length *= 256;
                    file_length += message[3];
                    file_transfer_state = 2;
                    display.clear_display();
                    char display_message[17];
                    sprintf(display_message,"F:%s",filename);
                    display.set_position(0,0);
                    display.write_string(display_message);
                    display.set_position(1,0);
                    sprintf(display_message,"S:%d b",file_length);
                    display.write_string(display_message);
                    block_index = 0;
                    //Work out how many blocks the file will be sent in (size = block_size, tested at 100 bytes)
                    //Allocate memory for the file up to a limit of 16 blocks; larger files will be split across
                    //multiple blocks....
                    final_block = file_length / block_size;
                    if(file_length % block_size != 0) final_block ++;
                    //int target_size = file_length;
                    //if(file_length > (block_size * 16)) target_size = block_size * 16;
                    //file_data = (char *) malloc(target_size);
                    psi.debug("File size %d bytes (%d blocks of %d bytes)\n",file_length,final_block,block_size);
                    ft_timeout.detach();
                    ft_timeout.attach(this,&SerialControl::IF_file_transfer_timeout,1.0);
                    //Send acknowledge (size of file)
                    bt.printf("%c%c%c%c%c",RESPONSE_MESSAGE_BYTE,3,message[1],message[2],message[3]);
                }
                break;
            case 2:
                block_index ++;
                display.clear_display();
                display.set_position(0,0);
                display.write_string("FILE TRANSFER");
                display.set_position(1,0);
                char details_string[17];
                sprintf(details_string,"BLOCK %d OF %d",block_index,final_block);
                display.write_string(details_string);
                expected_size = block_size;
                if(block_index == final_block) expected_size = file_length % block_size;
                if(expected_size == 0) expected_size = block_size;
                if(length!=expected_size + 1) {
                    // Unexpected length
                    psi.debug("File data unexpected length in packet %d (%d bytes received, %d bytes expected)\n",block_index,length-1,expected_size);
                } else {
                    char transfer_mode[2]= {'a'};
                    if(block_index == 1) {
                        transfer_mode[0]='w';
                    }
                    FILE *fp = fopen(filename,transfer_mode);
                    //strncpy(data_block,message+1,length);
                    //data_block[length]=0;
                    //fprintf(fp,data_block);
                    int bytes_written;
                    bytes_written = fwrite(message+1,expected_size,1,fp);
                    fclose(fp);
                    if(data_written == false && bytes_written > 0) data_written = true;
                    psi.debug("Bytes written: %d\n",expected_size * bytes_written);
                    if(block_index < final_block) {
                        psi.debug("Message packet %d received and written\n",block_index);
                        //Send acknowledge ("D")
                        ft_timeout.detach();
                        ft_timeout.attach(this,&SerialControl::IF_file_transfer_timeout,1.0);
                        bt.printf("%c%c%s",RESPONSE_MESSAGE_BYTE,1,"D");
                    } else {
                        //Last data block written
                        //[Put file checking code here]
                        //Send acknowledge ("P");
                        bt.printf("%c%c%s",RESPONSE_MESSAGE_BYTE,1,"F");
                        ft_timeout.detach();
                        psi.debug("File transfer completed successfully\n");
                        wait(0.25);
                        //Calculate CRC16 value for file
                        IF_calculateCRC16(file_length);

                        display.clear_display();
                        display.write_string("FILE TRANSFER");
                        display.set_position(1,0);
                        display.write_string("COMPLETE");
                        wait(1);
                        psi.debug("File transfer mode ended\n");
                        IF_end_file_transfer_mode();
                    }
                }
                break;
        }
    }
}


void SerialControl::IF_handle_user_serial_message(char * message, char length, char interface)
{
    char buffer[255];
    sprintf(buffer,message,length);
    for(int i=0; i<length; i++) {
        buffer[i]=message[i];
    }
    buffer[length]=0;
//    if(interface) debug("Received BT message:%s [%d chars]\n",buffer,length);
//    else debug("Received USB message:%s [%d chars]\n",buffer,length);
    handle_user_serial_message(message,length,interface);
}

void SerialControl::IF_handle_command_serial_message(char message[3], char interface)
{
    char iface [4];
    if(interface) strcpy(iface,"BT");
    else strcpy(iface,"USB");
    char command [26];
    char subcommand[30];
    float dec;
    float l_dec;
    float r_dec;
    int irp_delay;
    char colour_string[7];
    char ret_message[50];
    char send_message = 0;
    char command_status = 0;
    // command_status values:
    // 0 - unrecognised command
    // 1 - command actioned
    // 2 - command blocked
    // 3 - invalid parameters

    subcommand[0]=0;
    command[0]=0;
    switch(message[0]) {

            // MOTOR COMMANDS

        case 1:
            strcpy(command,"SET LEFT MOTOR");
            dec = IF_decode_float(message[1],message[2]);
            sprintf(subcommand,"%1.5f",dec);
            if(allow_commands) {
                command_status = 1;
                motors.set_left_motor_speed(dec);
            } else command_status = 2;
            break;
        case 2:
            strcpy(command,"SET RIGHT MOTOR");
            dec = IF_decode_float(message[1],message[2]);
            sprintf(subcommand,"%1.5f",dec);
            if(allow_commands) {
                motors.set_right_motor_speed(dec);
                command_status = 1;
            } else command_status = 2;
            break;
        case 3:
            strcpy(command,"SET BOTH MOTORS");
            dec = IF_decode_float(message[1],message[2]);
            sprintf(subcommand,"%1.5f",dec);
            if(allow_commands) {
                command_status = 1;
                motors.forward(dec);
            } else command_status = 2;
            break;
        case 4:
            strcpy(command,"BRAKE LEFT MOTOR");
            sprintf(subcommand,"");
            if(allow_commands) {
                command_status = 1;
                motors.brake_left_motor();
            } else command_status = 2;
            break;
        case 5:
            strcpy(command,"BRAKE RIGHT MOTOR");
            sprintf(subcommand,"");
            if(allow_commands) {
                command_status = 1;
                motors.brake_right_motor();
            } else command_status = 2;
            break;
        case 6:
            strcpy(command,"BRAKE BOTH MOTORS");
            sprintf(subcommand,"");
            if(allow_commands) {
                command_status = 1;
                motors.brake();
            } else command_status = 2;
            break;
        case 7:
            strcpy(command,"STOP BOTH MOTORS");
            sprintf(subcommand,"");
            if(allow_commands) {
                command_status = 1;
                motors.stop();
            } else command_status = 2;
            break;
        case 8:
            strcpy(command,"TURN ON SPOT");
            dec = IF_decode_float(message[1],message[2]);
            sprintf(subcommand,"%1.5f",dec);
            if(allow_commands) {
                command_status = 1;
                motors.turn(dec);
            } else command_status = 2;
            break;
        case 9:
            strcpy(command,"SET EACH MOTOR");
            l_dec = IF_decode_float(message[1]);
            r_dec = IF_decode_float(message[2]);
            sprintf(subcommand,"L=%1.3f R=%1.3f",l_dec,r_dec);
            if(allow_commands) {
                command_status = 1;
                motors.set_left_motor_speed(l_dec);
                motors.set_right_motor_speed(r_dec);
            } else command_status = 2;
            break;
            // LED COMMANDS

        case 10:
            strcpy(command,"SET LED STATES");
            sprintf(subcommand,"G:%s R:%s",IF_char_to_binary_char(message[1]), IF_char_to_binary_char(message[2]));
            if(allow_commands) {
                command_status = 1;
                led.set_leds(message[1],message[2]);
            } else command_status = 2;
            break;
        case 11:
            strcpy(command,"SET RED LED STATES");
            sprintf(subcommand,"%s",IF_char_to_binary_char(message[1]));
            if(allow_commands) {
                command_status = 1;
                led.set_red_leds(message[1]);
            } else command_status = 2;
            break;
        case 12:
            strcpy(command,"SET GREEN LED STATES");
            sprintf(subcommand,"%s",IF_char_to_binary_char(message[1]));
            if(allow_commands) {
                command_status = 1;
                led.set_green_leds(message[1]);
            } else command_status = 2;
            break;
        case 13:
            strcpy(command,"SET LED");
            switch(message[2]) {
                case 1:
                    strcpy(colour_string,"RED");
                    break;
                case 2:
                    strcpy(colour_string,"GREEN");
                    break;
                case 3:
                    strcpy(colour_string,"BOTH");
                    break;
                case 0:
                    strcpy(colour_string,"OFF");
                    break;
            }
            if(message[1] < 8 && message[2] < 4) {
                sprintf(subcommand,"%d %s",message[1],colour_string);
                if(allow_commands) {
                    command_status = 1;
                    led.set_led(message[1],message[2]);
                } else command_status = 2;
            } else {
                sprintf(subcommand,"[INVALID CODE]");
                command_status = 3;
            }
            break;
        case 14:
            strcpy(command,"SET CENTER LED STATE");
            switch(message[1]) {
                case 1:
                    strcpy(colour_string,"RED");
                    break;
                case 2:
                    strcpy(colour_string,"GREEN");
                    break;
                case 3:
                    strcpy(colour_string,"BOTH");
                    break;
                case 0:
                    strcpy(colour_string,"OFF");
                    break;
            }
            if(message[1] < 4) {
                sprintf(subcommand,"%s",colour_string);
                if(allow_commands) {
                    command_status = 1;
                    led.set_center_led(message[1]);
                } else command_status = 2;
            } else {
                sprintf(subcommand,"[INVALID CODE]");
                command_status = 3;
            }
            break;
        case 15:
            strcpy(command,"SET C.LED BRIGHTNESS");
            dec = IF_decode_unsigned_float(message[1],message[2]);
            sprintf(subcommand,"%1.5f",dec);
            if(allow_commands) {
                command_status = 1;
                led.set_center_led_brightness(dec);
            } else command_status = 2;
            break;
        case 16:
            strcpy(command,"SET MBED LEDS");
            sprintf(subcommand,"%s",IF_nibble_to_binary_char(message[1]));
            if(allow_commands) {
                command_status = 1;
                mbed_led1 = (message[1] & 128) >> 7;
                mbed_led2 = (message[1] & 64) >> 6;
                mbed_led3 = (message[1] & 32) >> 5;
                mbed_led4 = (message[1] & 16) >> 4;
            } else command_status = 2;
            break;
        case 17:
            strcpy(command,"BLINK OUTER LEDS");
            dec = IF_decode_unsigned_float(message[1],message[2]);
            sprintf(subcommand,"FOR %1.5fS",dec);
            if(allow_commands) {
                command_status = 1;
                led.blink_leds(dec);
            } else command_status = 2;
            break;
        case 18:
            strcpy(command,"SET BASE LED STATE");
            switch(message[1]) {
                case 1:
                    strcpy(subcommand,"ON");
                    break;
                case 0:
                    strcpy(subcommand,"OFF");
                    break;
            }
            if(allow_commands) {
                command_status = 1;
                led.set_base_led(message[1]);
            } else command_status = 2;
            break;
        case 19:
            strcpy(command,"SET CENTER LED ");
            switch(message[1]) {
                case 1:
                    strcpy(colour_string,"RED");
                    break;
                case 2:
                    strcpy(colour_string,"GREEN");
                    break;
                case 3:
                    strcpy(colour_string,"BOTH");
                    break;
                case 0:
                    strcpy(colour_string,"OFF");
                    break;
            }
            dec = IF_decode_unsigned_float(message[2]);
            sprintf(subcommand,"%s @ %1.5f brightness",colour_string,dec);
            if(allow_commands) {
                command_status = 1;
                led.set_center_led(message[1],dec);
            } else command_status = 2;
            break;

            // DISPLAY COMMANDS

        case 20:
            strcpy(command,"SET DISPLAY ");
            switch(message[1]) {
                case 0:
                    strcpy(subcommand,"CLEAR");
                    if(allow_commands) {
                        command_status = 1;
                        display.clear_display();
                    } else command_status = 2;
                    break;
                case 1:
                    strcpy(subcommand,"MESSAGE 1");
                    if(allow_commands) {
                        command_status = 1;
                        display.clear_display();
                        display.home();
                        display.write_string("PC CONNECTION");
                        display.set_position(1,0);
                        display.write_string("STARTED");
                    } else command_status = 2;
                    break;
                case 2:
                    strcpy(subcommand,"MESSAGE 2");
                    if(allow_commands) {
                        command_status = 1;
                        display.clear_display();
                        display.home();
                        display.write_string("PC CONNECTION");
                        display.set_position(1,0);
                        display.write_string("TERMINATED");
                    } else command_status = 2;
                    break;
                case 3:
                    strcpy(subcommand,"MESSAGE 3");
                    if(allow_commands) {
                        command_status = 1;
                        display.clear_display();
                        display.home();
                        display.write_string("ANDROID DEVICE");
                        display.set_position(1,0);
                        display.write_string("CONNECTED");
                    } else command_status = 2;
                    break;
                case 4:
                    strcpy(subcommand,"MESSAGE 4");
                    if(allow_commands) {
                        command_status = 1;
                        display.clear_display();
                        display.home();
                        display.write_string("ANDROID DEVICE");
                        display.set_position(1,0);
                        display.write_string("DISCONNECTED");
                    } else command_status = 2;
                    break;
                case 5:
                    strcpy(subcommand,"MESSAGE 5");
                    if(allow_commands) {
                        command_status = 1;
                        display.clear_display();
                        display.home();
                        display.write_string("PSI CONSOLE");
                        display.set_position(1,0);
                        display.write_string("CONNECTED");
                    } else command_status = 2;
                    break;
                case 6:
                    strcpy(subcommand,"MESSAGE 6");
                    if(allow_commands) {
                        command_status = 1;
                        display.clear_display();
                        display.home();
                        display.write_string("PSI CONSOLE");
                        display.set_position(1,0);
                        display.write_string("DISCONNECTED");
                    } else command_status = 2;
                    break;
            }
            break;
        case 21:
            strcpy(command,"SET CURSOR ");
            if(message[1] < 2 && message[2] < 16) {
                sprintf(subcommand,"[%d,%d]",message[1],message[2]);
                if(allow_commands) {
                    display.set_position(message[1],message[2]);
                } else command_status = 2;
            } else {
                sprintf(subcommand,"[INVALID]");
                command_status = 3;
            }
            break;
        case 22: {
            strcpy(command,"PRINT CHARACTERS ");
            char print_message[2];
            print_message[0]=message[1];
            print_message[1]=message[2];
            sprintf(subcommand,"[%c,%c]",message[1],message[2]);
            if(allow_commands) {
                display.write_string(print_message,2);
            } else command_status = 2;
            break;
        }
        case 23:
            strcpy(command,"SET DISPLAY B.NESS");
            dec = IF_decode_unsigned_float(message[1],message[2]);
            sprintf(subcommand,"%1.5f",dec);
            if(allow_commands) {
                command_status = 1;
                display.set_backlight_brightness(dec);
            } else command_status = 2;
            break;

        case 30:
            strcpy(command,"SET DEBUG MODE");
            switch(message[1]) {
                case 1:
                    strcpy(subcommand,"ON");
                    break;
                case 0:
                    strcpy(subcommand,"OFF");
                    break;
            }
            if(message[2] & 1) strcat (subcommand,"-PC");
            if(message[2] & 2) strcat (subcommand,"-BT");
            if(message[2] & 4) strcat (subcommand,"-DISP");
            if(allow_commands) {
                command_status = 1;
                debug_mode = message[1];
                debug_output = message[2];
            } else command_status = 2;
            break;
        case 31:
            strcpy(command,"SET DEMO MODE");
            switch(message[1] % 2) {
                case 1:
                    strcpy(subcommand,"ON");
                    break;
                case 0:
                    strcpy(subcommand,"OFF");
                    break;
            }
            if(allow_commands) {
                command_status = 1;
                demo_on = message[1] % 2;
                if(demo_on == 1) {
                    user_code_restore_mode = user_code_running;
                    user_code_running = 0;
                } else {
                    user_code_running = user_code_restore_mode;
                }
            } else command_status = 2;
            break;
        case 32:
            strcpy(command,"SET USER CODE");
            switch(message[1] % 2) {
                case 1:
                    strcpy(subcommand,"ON");
                    break;
                case 0:
                    strcpy(subcommand,"OFF");
                    break;
            }
            if(allow_commands) {
                command_status = 1;
                user_code_running = message[1] % 2;
            } else command_status = 2;
            break;
        case 33:
            strcpy(command,"PAUSE USER CODE");
            dec = IF_decode_unsigned_float(message[1],message[2]) * 10;
            sprintf(subcommand,"FOR %2.3fS",dec);
            if(allow_commands) {
                command_status = 1;
                psi.pause_user_code(dec);
            } else command_status = 2;
            break;

        case 34:
            strcpy(command,"RESET ENCODERS");
            if(allow_commands) {
                command_status = 1;
                psi.reset_encoders();
            } else command_status = 2;
            break;

        case 35:
            strcpy(command,"SET ALLOW COMMANDS");
            switch(message[1] % 2) {
                case 1:
                    strcpy(subcommand,"ON");
                    break;
                case 0:
                    strcpy(subcommand,"OFF");
                    break;
            }
            allow_commands = message[1] % 2;
            command_status = 1;
            break;

        case 36:
            irp_delay = (message[1] << 8) + message[2];
            sprintf(command,"SET IR PULSE DELAY %d MS",irp_delay);
            if(allow_commands) {
                command_status = 1;
                ir_pulse_delay = irp_delay;
            } else command_status = 2;
            break;
        case 37:
            irp_delay = (message[1] << 8) + message[2];
            sprintf(command,"SET BASE IR PULSE DELAY %d MS",irp_delay);
            if(allow_commands) {
                command_status = 1;
                base_ir_pulse_delay = irp_delay;
            } else command_status = 2;
            break;

            // MOTOR REQUESTS
        case 40:
            strcpy(command,"GET LEFT MOTOR SPEED");
            sprintf(ret_message,"%1.5f",motor_left_speed);
            send_message = 1;
            break;

        case 41:
            strcpy(command,"GET RIGHT MOTOR SPEED");
            sprintf(ret_message,"%1.5f",motor_right_speed);
            send_message = 1;
            break;
        case 42:
            strcpy(command,"GET BRAKE STATES");
            sprintf(ret_message,"%d,%d",motor_left_brake,motor_right_brake);
            send_message = 1;
            break;
        case 43:
            strcpy(command,"GET MOTOR STATES");
            //sprintf(ret_message,"%d,%d",motor_left_brake,motor_right_brake);
            send_message = 1;
            break;
        case 44:
            strcpy(command,"GET ENCODERS");
            sprintf(ret_message,"%d,%d",left_encoder,right_encoder);
            send_message = 1;
            break;

            // LED REQUESTS
        case 50:
            strcpy(command,"GET LED STATES");
            sprintf(ret_message,"%04x",led.get_led_states());
            send_message = 1;
            break;

            // GENERAL REQUESTS
        case 60:
            strcpy(command,"GET SOFTWARE VERSION");
            sprintf(ret_message,"%1.2f",SOFTWARE_VERSION_CODE);
            send_message = 1;
            break;

        case 61:
            strcpy(command,"GET UPTIME");
            sprintf(ret_message,"%6.2f",psi.get_uptime());
            send_message = 1;
            break;

        case 62:
            strcpy(command,"GET ID");
            sprintf(ret_message,"%d",robot_id);
            send_message = 1;
            break;

        case 63:
            strcpy(command,"GET SWITCH BYTE");
            sprintf(ret_message,"%02x",switch_byte);
            send_message = 1;
            break;
        case 64:
            strcpy(command,"GET USER CODE");
            sprintf(ret_message,"%d",user_code_running);
            send_message = 1;
            break;
        case 65:
            strcpy(command,"GET RESPONSE STRING");
            sprintf(ret_message,"PSI");
            send_message = 1;
            break;
        case 66:
            strcpy(command,"GET PROGRAM NAME");
            sprintf(ret_message,"%s",program_name);
            send_message = 1;
            break;
        case 67:
            strcpy(command,"GET AUTHOR NAME");
            sprintf(ret_message,"%s",author_name);
            send_message = 1;
            break;
        case 68:
            strcpy(command,"GET DEBUG MODE");
            sprintf(ret_message,"%1d%1d",debug_mode,debug_output);
            send_message = 1;
            break;
        case 69:
            strcpy(command,"GET SYSTEM WARNINGS");
            sprintf(ret_message,"%d",system_warnings);
            send_message = 1;
            break;


            // Sensors
        case 80:
            strcpy(command,"STORE BG. IR VALUES");
            if(allow_commands) {
                command_status = 1;
                sensors.store_background_raw_ir_values();
            } else command_status = 2;
            break;
        case 81:
            strcpy(command,"STORE IL. IR VALUES");
            if(allow_commands) {
                command_status = 1;
                sensors.store_illuminated_raw_ir_values();
            } else command_status = 2;
            break;
        case 82:
            strcpy(command,"STORE IR VALUES");
            if(allow_commands) {
                command_status = 1;
                sensors.store_ir_values();
            } else command_status = 2;
            break;
        case 83:
            strcpy(command,"STORE BG BASE IR VALUES");
            if(allow_commands) {
                command_status = 1;
                sensors.store_background_base_ir_values();
            } else command_status = 2;
            break;
        case 84:
            strcpy(command,"STORE IL. BASE IR VALUES");
            if(allow_commands) {
                command_status = 1;
                sensors.store_illuminated_base_ir_values();
            } else command_status = 2;
            break;
        case 85:
            strcpy(command,"STORE BASE IR VALUES");
            if(allow_commands) {
                command_status = 1;
                sensors.store_base_ir_values();
            } else command_status = 2;
            break;
        case 86:
            strcpy(command,"STORE ALL IR VALUES");
            if(allow_commands) {
                command_status = 1;
                sensors.store_ir_values();
                sensors.store_base_ir_values();
            } else command_status = 2;
            break;
        case 90:
            sprintf(command,"%s %d","GET BG IR VALUE",message[1]);
            sprintf(ret_message,"%d",sensors.get_background_raw_ir_value(message[1]));
            send_message = 1;
            break;
        case 91:
            sprintf(command,"%s %d","GET IL IR VALUE",message[1]);
            sprintf(ret_message,"%d",sensors.get_illuminated_raw_ir_value(message[1]));
            send_message = 1;
            break;
        case 92:
            strcpy(command,"GET BG IR VALUES");
            sprintf(ret_message,"%03X%03X%03X%03X%03X%03X%03X%03X",sensors.get_background_raw_ir_value(0),sensors.get_background_raw_ir_value(1),sensors.get_background_raw_ir_value(2),sensors.get_background_raw_ir_value(3),sensors.get_background_raw_ir_value(4),sensors.get_background_raw_ir_value(5),sensors.get_background_raw_ir_value(6),sensors.get_background_raw_ir_value(7));
            send_message = 1;
            break;
        case 93:
            strcpy(command,"GET ILLUMINATED IR VALUES");
            sprintf(ret_message,"%03X%03X%03X%03X%03X%03X%03X%03X",sensors.get_illuminated_raw_ir_value(0),sensors.get_illuminated_raw_ir_value(1),sensors.get_illuminated_raw_ir_value(2),sensors.get_illuminated_raw_ir_value(3),sensors.get_illuminated_raw_ir_value(4),sensors.get_illuminated_raw_ir_value(5),sensors.get_illuminated_raw_ir_value(6),sensors.get_illuminated_raw_ir_value(7));
            send_message = 1;
            break;
        case 94:
            sprintf(command,"%s %d","GET BG BASE IR VALUE",message[1]);
            sprintf(ret_message,"%d",sensors.get_background_base_ir_value(message[1]));
            send_message = 1;
            break;
        case 95:
            sprintf(command,"%s %d","GET IL BASE IR VALUE",message[1]);
            sprintf(ret_message,"%d",sensors.get_illuminated_base_ir_value(message[1]));
            send_message = 1;
            break;
        case 96:
            strcpy(command,"GET BG BASE IR VALUES");
            sprintf(ret_message,"%03X%03X%03X%03X%03X",sensors.get_background_base_ir_value(0),sensors.get_background_base_ir_value(1),sensors.get_background_base_ir_value(2),sensors.get_background_base_ir_value(3),sensors.get_background_base_ir_value(4));
            send_message = 1;
            break;
        case 97:
            strcpy(command,"GET IL BASE IR VALUES");
            sprintf(ret_message,"%03X%03X%03X%03X%03X",sensors.get_illuminated_base_ir_value(0),sensors.get_illuminated_base_ir_value(1),sensors.get_illuminated_base_ir_value(2),sensors.get_illuminated_base_ir_value(3),sensors.get_illuminated_base_ir_value(4));
            send_message = 1;
            break;
        case 98:
            strcpy(command,"CALCULATE BASE IR VALUES");
            sprintf(ret_message,"%03X%03X%03X%03X%03X",sensors.calculate_base_ir_value(0),sensors.calculate_base_ir_value(1),sensors.calculate_base_ir_value(2),sensors.calculate_base_ir_value(3),sensors.calculate_base_ir_value(4));
            send_message = 1;
            break;
        case 99:
            strcpy(command,"CALCULATE SIDE IR VALUES");
            sprintf(ret_message,"%03X%03X%03X%03X%03X%03X%03X%03X",sensors.calculate_side_ir_value(0),sensors.calculate_side_ir_value(1),sensors.calculate_side_ir_value(2),sensors.calculate_side_ir_value(3),sensors.calculate_side_ir_value(4),sensors.calculate_side_ir_value(5),sensors.calculate_side_ir_value(6),sensors.calculate_side_ir_value(7));
            send_message = 1;
            break;
        case 100:
            strcpy(command,"START FILE TRANSFER MODE");
            if(allow_commands) {
                command_status = 1;
                IF_start_file_transfer_mode();
                sprintf(ret_message,"OK");
                send_message = 1;
            } else command_status = 2;
            break;
        case 110:
            strcpy(command,"GET FIRMWARE VERSION");
            sprintf(ret_message,"%1.2f",firmware_version);
            send_message = 1;
            break;
        case 111:
            strcpy(command,"GET SERIAL NUMBER");
            sprintf(ret_message,"%2.3f",serial_number);
            send_message = 1;
            break;
        case 112:
            strcpy(command,"GET HAS SIDE IR");
            sprintf(ret_message,"%d",has_side_ir);
            send_message = 1;
            break;
        case 113:
            strcpy(command,"GET HAS BASE IR");
            sprintf(ret_message,"%d",has_base_ir);
            send_message = 1;
            break;
        case 114:
            strcpy(command,"GET HAS ENCODERS");
            sprintf(ret_message,"%d",has_wheel_encoders);
            send_message = 1;
            break;
        case 115:
            strcpy(command,"GET HAS AUDIO");
            sprintf(ret_message,"%d",has_audio_pic);
            send_message = 1;
            break;
        case 116:
            strcpy(command,"GET HAS RECHARGING");
            sprintf(ret_message,"%d",has_recharging_circuit);
            send_message = 1;
            break;
        case 117:
            strcpy(command,"GET HAS COMPASS");
            sprintf(ret_message,"%d",has_compass);
            send_message = 1;
            break;
        case 118:
            strcpy(command,"GET HAS ULTRASONIC");
            sprintf(ret_message,"%d",has_ultrasonic_sensor);
            send_message = 1;
            break;
        case 119:
            strcpy(command,"GET HAS TEMPERATURE");
            sprintf(ret_message,"%d",has_temperature_sensor);
            send_message = 1;
            break;
        case 120:
            strcpy(command,"GET HAS BASE COLOUR");
            sprintf(ret_message,"%d",has_base_colour_sensor);
            send_message = 1;
            break;
        case 121:
            strcpy(command,"GET HAS TOP COLOUR");
            sprintf(ret_message,"%d",has_top_colour_sensor);
            send_message = 1;
            break;
        case 122:
            strcpy(command,"GET HAS RADIO");
            sprintf(ret_message,"%d",has_433_radio);
            send_message = 1;
            break;
        case 123: {
            strcpy(command,"GET FIRMWARE H-DESC");
            char byte0 = 0;
            char byte1 = 1;
            if(has_side_ir)byte0+=128;
            if(has_base_ir)byte0+=64;
            if(has_wheel_encoders)byte0+=32;
            if(has_audio_pic)byte0+=16;
            if(has_recharging_circuit)byte0+=8;
            if(has_compass)byte0+=4;
            if(has_ultrasonic_sensor)byte0+=2;
            if(has_temperature_sensor)byte0+=1;
            if(has_base_colour_sensor)byte1+=128;
            if(has_top_colour_sensor)byte1+=64;
            if(has_433_radio)byte1+=32;
            sprintf(ret_message,"%c%c",byte0,byte1);
            send_message = 1;
            break;
        }
        case 124:
            strcpy(command,"GET PCB VERSION");
            sprintf(ret_message,"%1.2f",pcb_version);
            send_message = 1;
            break;
    }


    if(send_message) {
        char message_length = strlen(ret_message);
        switch(interface) {
            case 0:
                pc.printf("%c%c%s",RESPONSE_MESSAGE_BYTE,message_length,ret_message);
                break;
            case 1:
                bt.printf("%c%c%s",RESPONSE_MESSAGE_BYTE,message_length,ret_message);
                break;
        }
        psi.debug("Received %s request message: %s %s [%02x%02x%02x]\nReply: %s [%d ch]\n",iface, command, subcommand,message[0],message[1],message[2],ret_message,message_length);
    } else {
        switch(interface) {
            case 0:
                pc.printf("%c%c",ACKNOWLEDGE_MESSAGE_BYTE,command_status);
                break;
            case 1:
                bt.printf("%c%c",ACKNOWLEDGE_MESSAGE_BYTE,command_status);
                break;
        }
        switch(command_status) {
            case 0:
                psi.debug("Unrecognised %s command message [%02x%02x%02x]\n",iface,message[0],message[1],message[2]);
                break;
            case 1:
                psi.debug("Actioned %s command message:%s %s [%02x%02x%02x]\n",iface, command, subcommand,message[0],message[1],message[2]);
                break;
            case 2:
                psi.debug("Blocked %s command message:%s %s [%02x%02x%02x]\n",iface, command, subcommand,message[0],message[1],message[2]);
                break;
            case 3:
                psi.debug("Invalid %s command message:%s %s [%02x%02x%02x]\n",iface, command, subcommand,message[0],message[1],message[2]);
                break;
        }
    }
}

char * SerialControl::IF_nibble_to_binary_char(char in)
{
    char * ret = (char*)malloc(sizeof(char)*5);
    for(int i=0; i<4; i++) {
        if(in & (128 >> i)) ret[i]='1';
        else ret[i]='0';
    }
    ret[4]=0;
    return ret;
}

char * SerialControl::IF_char_to_binary_char(char in)
{
    char * ret = (char*)malloc(sizeof(char)*9);
    for(int i=0; i<8; i++) {
        if(in & (128 >> i)) ret[i]='1';
        else ret[i]='0';
    }
    ret[8]=0;
    return ret;
}

float SerialControl::IF_decode_unsigned_float(char byte0, char byte1)
{
    unsigned short sval = (byte0) << 8;
    sval += byte1;
    float scaled = sval / 65535.0f;
    return scaled;
}

float SerialControl::IF_decode_float(char byte0, char byte1)
{
    // MSB is byte 0 is sign, rest is linear spread between 0 and 1
    char sign = byte0 / 128;
    short sval = (byte0 % 128) << 8;
    sval += byte1;
    float scaled = sval / 32767.0f;
    if(sign == 0) scaled = 0-scaled;
    return scaled;
}

float SerialControl::IF_decode_unsigned_float(char byte0)
{
    unsigned short sval = (byte0);
    float scaled = sval / 255.0f;
    return scaled;
}

float SerialControl::IF_decode_float(char byte0)
{
    // MSB is byte 0 is sign, rest is linear spread between 0 and 1
    char sign = byte0 / 128;
    short sval = (byte0 % 128);
    float scaled = sval / 127.0f;
    if(sign == 0) scaled = 0-scaled;
    return scaled;
}


void SerialControl::IF_pc_rx_command_timeout()
{
    char message_array[6];
    char length = 1 + pc_command_message_byte;
    pc_command_message_started = 0;
    message_array[0] = COMMAND_MESSAGE_BYTE;
    for(int k=0; k<pc_command_message_byte; k++) {
        message_array[k+1] = pc_command_message[k];
    }
    IF_handle_user_serial_message(message_array, length, 0);
}

void SerialControl::IF_bt_rx_command_timeout()
{
    char message_array[6];
    char length = 1 + bt_command_message_byte;
    bt_command_message_started = 0;
    message_array[0] = COMMAND_MESSAGE_BYTE;
    for(int k=0; k<bt_command_message_byte; k++) {
        message_array[k+1] = bt_command_message[k];
    }
    IF_handle_user_serial_message(message_array, length, 1);
}

void SerialControl::IF_pc_rx_callback()
{
    int count = 0;
    char message_array[255];
    pc_command_timeout.detach();
    while(pc.readable()) {
        char tc = pc.getc();
        message_array[count] = tc;
        wait(0.01);
        //printf("input: %c and bite number: %i\n",message_array[count],count); //testing
        count ++;
        message_array[count]=0;
        /*if(pc_command_message_started == 1) {
            if(pc_command_message_byte == 3) {
                pc_command_timeout.detach();
                if(tc == COMMAND_MESSAGE_BYTE) {
                    // A complete command message succesfully received, call handler
                    pc_command_message_started = 0;
                    count = 0;
                    IF_handle_command_serial_message(pc_command_message , 0);
                } else {
                    // Message is not a valid command message as 5th byte is not correct; treat whole message as a user message
                    pc_command_message_started = 0;
                    message_array[0] = COMMAND_MESSAGE_BYTE;
                    message_array[1] = pc_command_message[0];
                    message_array[2] = pc_command_message[1];
                    message_array[3] = pc_command_message[2];
                    message_array[4] = tc;
                    count = 5;
                }
            } else {
                pc_command_message[pc_command_message_byte] = tc;
                pc_command_message_byte ++;
            }
        } else {
            if(count == 1) {
                if(tc == COMMAND_MESSAGE_BYTE) {
                    pc_command_timeout.attach(this,&SerialControl::IF_pc_rx_command_timeout,command_timeout_period);
                    pc_command_message_started = 1;
                    pc_command_message_byte = 0;

                }
            }
        }*/
        //printf("total input : %s\n",message_array); //testing 
    }
    if(/*!pc_command_message_started && count>0*/1) IF_handle_user_serial_message(message_array, count, 0);
}

Timeout bt_message_timeout;
//static float bt_message_timeout_period = 0.001; // 1 millisecond
char bt_buffer[255];
int bt_buffer_index = 0;

void SerialControl::IF_bt_message_timeout()
{
    char buffer[255];

    sprintf(buffer, bt_buffer, bt_buffer_index);
    buffer[bt_buffer_index] = 0;
    if(file_transfer_mode == 1) {
        IF_handle_file_transfer_serial_message(bt_buffer, bt_buffer_index, 1);
    } else {
//    debug("BT message timeout: %s [%d chars]\n", buffer, bt_buffer_index);
        if(bt_buffer_index == 5 && buffer[0] == COMMAND_MESSAGE_BYTE && buffer[4] == COMMAND_MESSAGE_BYTE) {
            bt_command_message[0] = buffer[1];
            bt_command_message[1] = buffer[2];
            bt_command_message[2] = buffer[3];
            IF_handle_command_serial_message(bt_command_message , 1);
        } else IF_handle_user_serial_message(bt_buffer, bt_buffer_index, 1);
    }
    bt_buffer_index = 0;
}


//void IF_bt_rx_callback()
//{
//    while(bt.readable()) {
//        char byte = bt.getc();
//
//        bt_buffer[bt_buffer_index] = byte;
//        bt_buffer_index++;
//    }
//
//    bt_message_timeout.attach(&IF_bt_message_timeout, bt_message_timeout_period);
//}


void SerialControl::IF_set_filename(char * filename_in)
{
    strcpy(filename,filename_in);
}

unsigned short SerialControl::IF_calculateCRC16(int file_length)
{
    unsigned short crc16table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    //Opens, reads and calculates the CRC16 value for file pointed to by filename
    unsigned short crc_value = 0;
    FILE *fp = fopen(filename,"r");
    char * buffer;
    int limit = 512;
    if(file_length < 512) limit = file_length;
    buffer = (char*) malloc (sizeof(char)*limit);
    int blocks = 1;
    if(file_length > limit) blocks += file_length / limit;
    for(int i=0; i<blocks; i++) {
        //Determine size of this block
        int blocksize = limit;
        if(i == blocks-1) {
            if((file_length % limit) != 0) blocksize = file_length % limit;
        }
        psi.debug("Calculating %d bytes of CRC data...\n",blocksize);
        int result;
        result = fread(buffer,1,blocksize,fp);
        psi.debug("Data read: %d\n",result);
        for(int j=0; j<blocksize; j++) {
            int subindex = ((crc_value>>8)^*(char *)(buffer[j]))&0x00FF;
            //debug("J:%d Subindex:%d\n",j,subindex);
            unsigned short table_value = crc16table[subindex];
            crc_value=(crc_value<<8)^table_value;
        }
    }
    fclose(fp);
    psi.debug("CRC Calculated: %x\n",crc_value);
    return crc_value;
}

void SerialControl::IF_bt_rx_callback()
{
    int count = 0;
    char message_array[255];

    wait_ms(500); // Wait 0.5ms to allow a complete message to arrive before atttempting to process it

    while(bt.readable()) {
        char tc = bt.getc();
        message_array[count] = tc;
        count ++;
        if(bt_command_message_started == 1) {
            if(bt_command_message_byte == 3) {
                bt_command_timeout.detach();
                if(tc == COMMAND_MESSAGE_BYTE) {
                    // A complete command message succesfully received, call handler
                    bt_command_message_started = 0;
                    count = 0;
                    IF_handle_command_serial_message(bt_command_message , 1);
                } else {
                    // Message is not a valid command message as 5th byte is not correct; treat whole message as a user message
                    bt_command_message_started = 0;
                    message_array[0] = COMMAND_MESSAGE_BYTE;
                    message_array[1] = bt_command_message[0];
                    message_array[2] = bt_command_message[1];
                    message_array[3] = bt_command_message[2];
                    message_array[4] = tc;
                    count = 5;
                }
            } else {
                bt_command_timeout.attach(this,&SerialControl::IF_bt_rx_command_timeout,command_timeout_period);
                bt_command_message[bt_command_message_byte] = tc;
                bt_command_message_byte ++;
            }
        } else {
            if(count == 1) {
                if(tc == COMMAND_MESSAGE_BYTE) {
                    bt_command_message_started = 1;
                    bt_command_message_byte = 0;

                }
            }
        }
    }
    if(!bt_command_message_started && count>0) IF_handle_user_serial_message(message_array, count, 1);
}