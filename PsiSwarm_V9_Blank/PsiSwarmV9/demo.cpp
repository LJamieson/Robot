/* University of York Robotics Laboratory PsiSwarm Library: Demo Mode Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: demo.cpp
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * Version 0.9 : Added colour sensor functions, colour sensor demo and self-test mode
 * Version 0.8 : Rewritten as OO\C++
 * Version 0.7 : Updated for new MBED API
 * Version 0.5 : Added motor calibration menu
 * Version 0.4 : Added PsiSwarmBasic menu
 * Version 0.2 : Remove most the functionality from center-button push to allow all operations to be accessable from
 *               four directions alone.
 *               Added extra sensor information, added various testing demos
 *
 *
 * June 2017
 *
 */


#include "psiswarm.h"

// PID terms
#define LF_P_TERM 0.2
#define LF_I_TERM 0
#define LF_D_TERM 4

char quick_test = 0;
char top_menu = 0;
char sub_menu = 0;
char level = 0;
char started = 0;
char topline[17];
char bottomline[17];
char led_state[9];
char all_led_state = 0;
char base_led_state = 0;
char brightness = 20;
char bl_brightness = 100;
char base_ir_index = 0;
char side_ir_index = 0;
signed short left_speed = 0;
signed short right_speed = 0;
char both_motor_mode = 0;
char last_switch_pressed;
Timeout demo_event;
char handling_event = 0;

Timeout demo_timeout;
char demo_running = 0;
Timer demo_timer;
float time_out;
float speed;
char state;
char led_step = 0;
char spin_step = 0;
char stress_step = 0;


float lf_right;
float lf_left;
float lf_current_pos_of_line = 0.0;
float lf_previous_pos_of_line = 0.0;
float lf_derivative,lf_proportional,lf_integral = 0;
float lf_power;
float lf_speed = 0.4;



void Demo::start_demo_mode()
{
    psi.debug("- Starting Demo Mode\n");
    if(use_flash_basic == 1) top_menu = 8;
    if((has_base_ir == 1 && base_ir_calibration_set != 1) || (has_base_colour_sensor == 1 && base_colour_calibration_set != 1)) top_menu = 7;
    demo_on = 1;
    display.set_backlight_brightness(bl_brightness * 0.01f);
    display.clear_display();
    display.write_string("PSI SWARM SYSTEM");
    display.set_position(1,0);
    display.write_string("   DEMO MODE");
    wait(0.5);
    display.clear_display();
    display.write_string("Use cursor to");
    display.set_position(1,0);
    display.write_string("navigate menus");
    char step = 0;
    while(demo_on) {
        if(demo_running == 0) {
            switch(step) {
                case 0:
                    mbed_led1 = 1;
                    mbed_led2 = 0;
                    break;
                case 1:
                    mbed_led2 = 1;
                    mbed_led1 = 0;
                    break;
            }
            step++;
            if(step==2)step=0;
        } else {
            mbed_led1 = 0;
            mbed_led2 = 0;
            mbed_led3 = 0;
            mbed_led4 = 0;
        }
        wait(0.5);
    }
    psi.debug("- Demo mode ended\n");
}

void Demo::demo_handle_switch_event(char switch_pressed)
{
    if(!handling_event) {
        handling_event = 1;
        last_switch_pressed = switch_pressed;
        demo_event.attach_us(this,&Demo::demo_event_thread, 1000);
    }
}

void Demo::demo_event_thread()
{
    handling_event = 0;
    if(started == 1) {
        switch(last_switch_pressed) {
            case 1: //Up pressed
                switch(level) {
                    case 0:
                        if(top_menu != 9 && top_menu!= 0) {
                            level++;
                            sub_menu = 0;
                        } else {
                            if(top_menu == 0) {
                                toggle_quick_test();
                            } else {
                                demo_on = 0;
                                user_code_running = user_code_restore_mode;
                            }
                        }
                        break;
                    case 1:
                        switch(top_menu) {
                            case 8: // PsiBasic Menu
                                if(sub_menu == psi_basic_file_count) level = 0;
                                break;
                            case 1: //LED Menu
                                if(sub_menu < 9) {
                                    if(led_state[sub_menu] == 0) led_state[sub_menu] = 3;
                                    else led_state[sub_menu]--;
                                    demo_update_leds();
                                }
                                if(sub_menu == 9) {
                                    if(all_led_state == 0) all_led_state = 3;
                                    else all_led_state--;
                                    for(int i=0; i<9; i++) {
                                        led_state[i]=all_led_state;
                                    }
                                    demo_update_leds();
                                }
                                if(sub_menu == 10) {
                                    base_led_state = 1 - base_led_state;
                                    demo_update_leds();
                                }
                                if(sub_menu == 11) {
                                    switch(brightness) {
                                        case 100:
                                            brightness = 50;
                                            break;
                                        case 2:
                                            brightness = 1;
                                            break;
                                        case 5:
                                            brightness = 2;
                                            break;
                                        case 10:
                                            brightness = 5;
                                            break;
                                        case 20:
                                            brightness = 10;
                                            break;
                                        case 50:
                                            brightness = 20;
                                            break;
                                    }
                                    demo_update_leds();
                                }
                                if(sub_menu == 12) {
                                    if(bl_brightness > 0) bl_brightness-=10;
                                    display.set_backlight_brightness(bl_brightness * 0.01f);
                                }
                                if(sub_menu == 13) level = 0;
                                break;
                            case 2: // Sensors Menu
                                if(sub_menu == 4 || sub_menu == 5 || sub_menu==6) {
                                    if(base_ir_index == 0) base_ir_index = 4;
                                    else base_ir_index --;
                                }
                                if(sub_menu > 6 && sub_menu < 10) {
                                    if(side_ir_index == 0) side_ir_index = 7;
                                    else side_ir_index --;
                                }
                                if(sub_menu == 13) level = 0;
                                break;
                            case 3: // Motor Menu
                                if(sub_menu == 0) {
                                    left_speed += 5;
                                    if(left_speed > 100) left_speed = 100;
                                    motors.set_left_motor_speed(left_speed / 100.0f);
                                }
                                if(sub_menu == 1) {
                                    right_speed += 5;
                                    if(right_speed > 100) right_speed = 100;
                                    motors.set_right_motor_speed(right_speed / 100.0f);
                                }
                                if(sub_menu == 2) {
                                    if(both_motor_mode == 0) both_motor_mode=5;
                                    else both_motor_mode--;
                                    switch(both_motor_mode) {
                                        case 0:
                                            motors.stop();
                                            break;
                                        case 1:
                                            motors.brake();
                                            break;
                                        case 2:
                                            motors.forward(0.5);
                                            break;
                                        case 3:
                                            motors.forward(1);
                                            break;
                                        case 4:
                                            motors.backward(0.5);
                                            break;
                                        case 5:
                                            motors.backward(1.0);
                                            break;
                                    }
                                }
                                if(sub_menu == 3) {
                                    level = 0;
                                }
                                break;
                            case 4: // Radio Menu
                                if(sub_menu == 0) level = 0;
                                break;
                            case 5: // Info Menu
                                if(sub_menu == 6) level = 0;
                                break;
                            case 7: // Calibration Menu
                                if(sub_menu == 0) sensors.calibrate_base_sensors();
                                if(sub_menu == 1) motors.calibrate_motors();
                                if(sub_menu == 2) level = 0;
                                break;
                            case 6: // Demo Menu
                                if(sub_menu == 0) {
                                    if(demo_running == 0) {
                                        start_line_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 1) {
                                    if(demo_running == 0) {
                                        start_obstacle_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 2) {
                                    if(demo_running == 0) {
                                        start_spinning_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 3) {
                                    if(demo_running == 0) {
                                        start_stress_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 4) {
                                    if(demo_running == 0) {
                                        start_colour_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 5) level = 0;
                                break;
                        }
                        break;
                }
                break;
            case 2: //Down pressed
                switch(level) {
                    case 0:
                        if(top_menu != 9 && top_menu != 0) {
                            level++;
                            sub_menu = 0;
                        } else {
                            if(top_menu == 0) {
                                toggle_quick_test();
                            } else {
                                demo_on = 0;
                                user_code_running = user_code_restore_mode;
                            }
                        }
                        break;
                    case 1:
                        switch(top_menu) {
                            case 8: // PsiBasic Menu
                                if(sub_menu == psi_basic_file_count) level = 0;
                                break;
                            case 1: //LED Menu
                                if(sub_menu < 9) {
                                    led_state[sub_menu]++;
                                    if(led_state[sub_menu] == 4) led_state[sub_menu] = 0;
                                    demo_update_leds();
                                }
                                if(sub_menu == 9) {
                                    all_led_state++;
                                    if(all_led_state == 4) all_led_state = 0;
                                    for(int i=0; i<9; i++) {
                                        led_state[i]=all_led_state;
                                    }
                                    demo_update_leds();
                                }
                                if(sub_menu == 10) {
                                    base_led_state = 1 - base_led_state;
                                    demo_update_leds();
                                }
                                if(sub_menu == 11) {
                                    switch(brightness) {
                                        case 1:
                                            brightness = 2;
                                            break;
                                        case 2:
                                            brightness = 5;
                                            break;
                                        case 5:
                                            brightness = 10;
                                            break;
                                        case 10:
                                            brightness = 20;
                                            break;
                                        case 20:
                                            brightness = 50;
                                            break;
                                        case 50:
                                            brightness = 100;
                                            break;
                                    }
                                    demo_update_leds();
                                }

                                if(sub_menu == 12) {
                                    if(bl_brightness < 100) bl_brightness+=10;
                                    display.set_backlight_brightness(bl_brightness * 0.01f);
                                }
                                if(sub_menu == 13) level = 0;

                                break;
                            case 2: // Sensors Menu
                                if(sub_menu == 4 || sub_menu == 5 || sub_menu == 6) {
                                    if(base_ir_index == 4) base_ir_index = 0;
                                    else base_ir_index ++;
                                }
                                if(sub_menu > 6 && sub_menu < 10) {
                                    if(side_ir_index == 7) side_ir_index = 0;
                                    else side_ir_index ++;
                                }
                                if(sub_menu == 13) level = 0;
                                break;
                            case 3: // Motor Menu
                                if(sub_menu == 0) {
                                    left_speed -= 5;
                                    if(left_speed < -100) left_speed = -100;
                                    motors.set_left_motor_speed(left_speed / 100.0f);
                                }
                                if(sub_menu == 1) {
                                    right_speed -= 5;
                                    if(right_speed < -100) right_speed = -100;
                                    motors.set_right_motor_speed(right_speed / 100.0f);
                                }
                                if(sub_menu == 2) {
                                    both_motor_mode++;
                                    if(both_motor_mode == 6) both_motor_mode=0;
                                    switch(both_motor_mode) {
                                        case 0:
                                            motors.stop();
                                            break;
                                        case 1:
                                            motors.brake();
                                            break;
                                        case 2:
                                            motors.forward(0.5);
                                            break;
                                        case 3:
                                            motors.forward(1);
                                            break;
                                        case 4:
                                            motors.backward(0.5);
                                            break;
                                        case 5:
                                            motors.backward(1.0);
                                            break;
                                    }
                                }
                                if(sub_menu == 3) {
                                    level = 0;
                                }
                                break;
                            case 4: // Radio Menu
                                if(sub_menu == 0) level = 0;
                                break;
                            case 5: // Info Menu
                                if(sub_menu == 6) level = 0;
                                break;
                            case 7: // Calibration Menu
                                if(sub_menu == 0) sensors.calibrate_base_sensors();
                                if(sub_menu == 1) motors.calibrate_motors();
                                if(sub_menu == 2) level = 0;
                                break;
                            case 6: // Demo Menu
                                if(sub_menu == 0) {
                                    if(demo_running == 0) {
                                        start_line_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 1) {
                                    if(demo_running == 0) {
                                        start_obstacle_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();

                                    }
                                }
                                if(sub_menu == 2) {
                                    if(demo_running == 0) {
                                        start_spinning_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 3) {
                                    if(demo_running == 0) {
                                        start_stress_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 4) {
                                    if(demo_running == 0) {
                                        start_colour_demo();
                                    } else {
                                        demo_running = 0;
                                        display.set_backlight_brightness(bl_brightness * 0.01f);
                                        motors.stop();
                                    }
                                }
                                if(sub_menu == 5) level = 0;
                                break;
                        }
                        break;
                }
                break;
            case 4: //Left pressed
                switch(level) {
                    case 0:
                        if(top_menu == 0) {
                            top_menu = 9;
                        } else {
                            top_menu --;
                            if(use_flash_basic == 0 && top_menu == 8) top_menu = 7;
                            if(has_433_radio == 0 && top_menu == 4) top_menu = 3;
                        }
                        break;
                    case 1:
                        switch(top_menu) {
                            case 1: //LED Menu
                                if(sub_menu == 0) sub_menu = 13;
                                else sub_menu --;
                                break;
                            case 2: //Sensors Menu
                                if(sub_menu == 0) sub_menu = 13;
                                else sub_menu --;
                                break;

                            case 3: //Motor Menu
                                if(sub_menu == 0) sub_menu = 3;
                                else sub_menu --;
                                break;
                            case 5: //Info Menu
                                if(sub_menu == 0) sub_menu = 6;
                                else sub_menu --;
                                break;
                            case 6: //Demo Menu
                                if(sub_menu == 0) sub_menu = 5;
                                else sub_menu --;
                                break;
                            case 7: //Calibrate Menu
                                if(sub_menu == 0) sub_menu = 2;
                                else sub_menu --;
                                break;
                            case 8: //PsiBasic Menu
                                if(sub_menu == 0) sub_menu = psi_basic_file_count;
                                else sub_menu --;
                        }
                        break;

                }
                break;
            case 8: //Right pressed
                switch(level) {
                    case 0:
                        top_menu ++;
                        if(top_menu == 8 && use_flash_basic == 0) top_menu = 9;
                        if(has_433_radio == 0 && top_menu == 4) top_menu = 5;
                        if((top_menu) > 9) top_menu = 0;
                        break;
                    case 1:
                        switch(top_menu) {
                            case 1: //LED Menu
                                if(sub_menu == 13) sub_menu = 0;
                                else sub_menu ++;
                                break;
                            case 2: //Sensors Menu
                                if(sub_menu == 13) sub_menu = 0;
                                else sub_menu ++;
                                break;
                            case 3: //Motor Menu
                                if(sub_menu == 3) sub_menu = 0;
                                else sub_menu ++;
                                break;
                            case 5: //Info Menu
                                if(sub_menu == 6) sub_menu = 0;
                                else sub_menu ++;
                                break;
                            case 6: //Demo Menu
                                if(sub_menu == 5) sub_menu = 0;
                                else sub_menu ++;
                                break;
                            case 7: //Calibrate Menu
                                if(sub_menu == 2) sub_menu = 0;
                                else sub_menu ++;
                                break;
                            case 8: //PsiBasic Menu
                                if(sub_menu == psi_basic_file_count) sub_menu = 0;
                                else sub_menu ++;
                        }
                        break;
                }
                break;
        }
    } else started = 1;
    display.clear_display();
    switch(level) {
        case 0:
            //Top level menu
            switch(top_menu) {
                case 0:
                    strcpy(topline,"---QUICK TEST---");
                    break;
                case 1:
                    strcpy(topline,"---TEST LEDS----");
                    break;
                case 2:
                    strcpy(topline,"--TEST SENSORS--");
                    break;
                case 3:
                    strcpy(topline,"--TEST MOTORS---");
                    break;
                case 4:
                    strcpy(topline,"---TEST RADIO---");
                    break;
                case 5:
                    strcpy(topline,"------INFO------");
                    break;
                case 6:
                    strcpy(topline,"---CODE DEMOS---");
                    break;
                case 7:
                    strcpy(topline,"---CALIBRATION--");
                    break;
                case 8:
                    strcpy(topline,"---PSI BASIC----");
                    break;
                case 9:
                    strcpy(topline,"------EXIT------");
                    break;
            }
            strcpy(bottomline,"");
            break;
        case 1:
            //Sub level menu
            switch(top_menu) {
                case 8:
                    strcpy(topline,"-PSI BASIC MENU-");
                    if(sub_menu == psi_basic_file_count) strcpy(bottomline,"EXIT");
                    else strcpy(bottomline,basic_filenames.at(sub_menu).c_str());
                    break;
                case 1:
                    strcpy(topline,"----LED MENU----");
                    char led_status[7];
                    if(sub_menu<9) {
                        switch(led_state[sub_menu]) {
                            case 0:
                                strcpy(led_status,"OFF");
                                break;
                            case 1:
                                strcpy(led_status,"RED");
                                break;
                            case 2:
                                strcpy(led_status,"GREEN");
                                break;
                            case 3:
                                strcpy(led_status,"YELLOW");
                                break;
                        }
                    }
                    if(sub_menu < 8) sprintf(bottomline,"OUTER %d: %s",(sub_menu + 1),led_status);
                    if(sub_menu == 8) sprintf(bottomline,"CENTER: %s",led_status);
                    if(sub_menu == 9) {
                        switch(all_led_state) {
                            case 0:
                                strcpy(led_status,"OFF");
                                break;
                            case 1:
                                strcpy(led_status,"RED");
                                break;
                            case 2:
                                strcpy(led_status,"GREEN");
                                break;
                            case 3:
                                strcpy(led_status,"YELLOW");
                                break;
                        }
                        sprintf(bottomline,"SET ALL %s", led_status);
                    }
                    if(sub_menu == 10) {
                        if(base_led_state == 0) strcpy(led_status,"OFF");
                        else strcpy(led_status,"ON");
                        sprintf(bottomline,"BASE: %s",led_status);
                    }
                    if(sub_menu == 11) sprintf(bottomline,"CLED BNESS %d%%", brightness);
                    if(sub_menu == 12) sprintf(bottomline,"DISP BNESS %d%%", bl_brightness);
                    if(sub_menu == 13) strcpy(bottomline,"EXIT");
                    break;

                case 2:
                    strcpy(topline,"--SENSORS MENU--");
                    switch(sub_menu) {
                        case 0: {
                            float battery = sensors.get_battery_voltage ();
                            sprintf(bottomline,"BATTERY: %1.3fV",battery);
                            break;
                        }
                        case 1: {
                            float dc = sensors.get_dc_voltage ();
                            sprintf(bottomline,"DC: %1.3fV",dc);
                            break;
                        }
                        case 2: {
                            float current = sensors.get_current ();
                            sprintf(bottomline,"CURRENT: %1.3fA",current);
                            break;
                        }
                        case 3: {
                            float temperature = sensors.get_temperature();
                            sprintf(bottomline,"TEMP: %3.2fC", temperature);
                            break;
                        }
                        case 4:
                            sensors.store_background_base_ir_values();
                            sprintf(bottomline,"BIR%dB:%d",base_ir_index+1,sensors.get_background_base_ir_value(base_ir_index));
                            break;
                        case 5:
                            sensors.store_illuminated_base_ir_values();
                            sprintf(bottomline,"BIR%dR:%d",base_ir_index+1,sensors.get_illuminated_base_ir_value(base_ir_index));
                            break;
                        case 6:
                            sensors.store_illuminated_base_ir_values();
                            sprintf(bottomline,"BIC%dR:%1.3f",base_ir_index+1,sensors.get_calibrated_base_ir_value(base_ir_index));
                            break;
                        case 7:
                            sensors.store_background_raw_ir_values();
                            sprintf(bottomline,"SIR%dB:%d",side_ir_index+1,sensors.get_background_raw_ir_value(side_ir_index));
                            break;
                        case 8:
                            sensors.store_illuminated_raw_ir_values();
                            sprintf(bottomline,"SIR%dR:%d",side_ir_index+1,sensors.get_illuminated_raw_ir_value(side_ir_index));
                            break;
                        case 9:
                            sprintf(bottomline,"SIR%dD:%3.1f",side_ir_index+1,0.0);
                            break;
                        case 10:
                            if(ultrasonic_distance_updated == 1) {
                                sprintf(bottomline,"USONIC:%3dcm",ultrasonic_distance);
                            } else sprintf(bottomline,"USONIC:---------");
                            sensors.update_ultrasonic_measure();
                            break;
                        case 11:
                            sensors.store_line_position();
                            if(line_found == 1)sprintf(bottomline,"LINE:%1.3f",line_position);
                            else sprintf(bottomline,"LINE:---------");
                            break;

                        case 12:
                            sprintf(bottomline,"COLOUR: %s",colour.get_colour_string(colour.detect_colour_once()));
                            break;
                        case 13:
                            sprintf(bottomline,"EXIT");
                            break;
                    }
                    break;
                case 3:
                    strcpy(topline,"--MOTORS MENU---");
                    switch(sub_menu) {
                        case 0:
                            sprintf(bottomline,"LEFT: %d%%", left_speed);
                            break;
                        case 1:
                            sprintf(bottomline,"RIGHT: %d%%", right_speed);
                            break;
                        case 2:
                            char both_mode_string[16];
                            switch(both_motor_mode) {
                                case 0:
                                    strcpy(both_mode_string,"OFF");
                                    break;
                                case 1:
                                    strcpy(both_mode_string,"BRAKE");
                                    break;
                                case 2:
                                    strcpy(both_mode_string,"+50%");
                                    break;
                                case 3:
                                    strcpy(both_mode_string,"+100%");
                                    break;
                                case 4:
                                    strcpy(both_mode_string,"-50%");
                                    break;
                                case 5:
                                    strcpy(both_mode_string,"-100%");
                                    break;
                            }
                            sprintf(bottomline,"BOTH TO %s",both_mode_string);
                            break;
                        case 3:
                            sprintf(bottomline,"EXIT");
                            break;
                    }
                    break;
                case 4:
                    strcpy(topline,"---RADIO MENU---");
                    switch(sub_menu) {

                        case 0:
                            sprintf(bottomline,"EXIT");
                            break;
                    }
                    break;
                case 5:
                    strcpy(topline,"---INFO MENU----");
                    switch(sub_menu) {
                        case 0:
                            sprintf(bottomline,"ROBOT ID: %d",robot_id);
                            break;
                        case 1:
                            sprintf(bottomline,"SOFTWARE: %1.2f",SOFTWARE_VERSION_CODE);
                            break;
                        case 2:
                            if(firmware_version > 0) sprintf(bottomline,"FIRMWARE: %1.2f",firmware_version);
                            else sprintf(bottomline,"FIRMWARE: ?????");
                            break;
                        case 3:
                            sprintf(bottomline,"PROG:%s",program_name);
                            break;
                        case 4:
                            sprintf(bottomline,"AUTH:%s",author_name);
                            break;
                        case 5:
                            sprintf(bottomline,"VER:%s",version_name);
                            break;
                        case 6:
                            sprintf(bottomline,"EXIT");
                            break;
                    }
                    break;
                case 6:
                    strcpy(topline,"---CODE DEMOS---");
                    switch(sub_menu) {
                        case 0:
                            sprintf(bottomline,"LINE FOLLOW");
                            break;
                        case 1:
                            sprintf(bottomline,"OBST. AVOID");
                            break;
                        case 2:
                            sprintf(bottomline,"COLOUR SPIN");
                            break;
                        case 3:
                            sprintf(bottomline,"STRESS TEST");
                            break;
                        case 4:
                            sprintf(bottomline,"COLOUR WALK");
                            break;
                        case 5:
                            sprintf(bottomline,"EXIT");
                            break;
                    }
                    break;
                case 7:
                    strcpy(topline,"---CALIB. MENU--");
                    switch(sub_menu) {
                        case 0:
                            sprintf(bottomline,"BASE SENSOR");
                            break;
                        case 1:
                            sprintf(bottomline,"MOTOR");
                            break;
                        case 2:
                            sprintf(bottomline,"EXIT");
                            break;
                    }
                    break;
            }
            break;
    }
    display.write_string(topline);
    display.set_position(1,0);
    display.write_string(bottomline);
    // Periodically update when on sensors menu
    if(top_menu == 2  && level == 1) {
        demo_event.attach(this,&Demo::demo_event_thread, 0.25);
    }
}

char test_step = 0;
char test_substep = 0;
char test_warnings = 0;


void Demo::quick_test_cycle()
{
    char next_step [] = {4,5,8,3};
    char test_message [17];
    int wait_period = SELF_TEST_PERIOD * 10;

    if(test_substep == 0) {
        display.clear_display();
        switch(test_step) {
            case 0:
                display.write_string("01 - Power      ");
                pc.printf("\nTest 01: Power quick tests [%f]\n",psi.get_uptime());
                break;
            case 1:
                display.write_string("02 - Base IR    ");
                pc.printf("\nTest 02: Base infrared tests [%f]\n",psi.get_uptime());
                break;
            case 2:
                display.write_string("03 - Side IR    ");
                pc.printf("\nTest 03: Side infrared tests [%f]\n",psi.get_uptime());
                break;
            case 3:
                display.write_string("04 - LEDs       ");
                pc.printf("\nTest 04: LED quick tests [%f]\n",psi.get_uptime());
                break;
        }
    }
    if(test_step == 1) {
        //Base IR tests
        sensors.store_background_base_ir_values();
        wait(0.05);
        sensors.store_illuminated_base_ir_values();
        pc.printf("Sample %d     1: %04d-%04d-%1.2f  2: %04d-%04d-%1.2f  3: %04d-%04d-%1.2f  4: %04d-%04d-%1.2f  5: %04d-%04d-%1.2f\n", (test_substep+1),
                  sensors.get_background_base_ir_value(0),           sensors.get_illuminated_base_ir_value(0), sensors.get_calibrated_base_ir_value (0),
                  sensors.get_background_base_ir_value(1),           sensors.get_illuminated_base_ir_value(1),sensors.get_calibrated_base_ir_value (1),
                  sensors.get_background_base_ir_value(2),           sensors.get_illuminated_base_ir_value(2),sensors.get_calibrated_base_ir_value (2),
                  sensors.get_background_base_ir_value(3),           sensors.get_illuminated_base_ir_value(3),sensors.get_calibrated_base_ir_value (3),
                  sensors.get_background_base_ir_value(4),           sensors.get_illuminated_base_ir_value(4),sensors.get_calibrated_base_ir_value (4));
        sprintf(test_message,"%d:%4d-%4d-%1.2f",test_substep+1,sensors.get_background_base_ir_value(test_substep),sensors.get_illuminated_base_ir_value(test_substep),sensors.get_calibrated_base_ir_value(test_substep));
        wait_us(SELF_TEST_PERIOD);
    }
    if(test_step == 2) {
        //Side IR tests
        sensors.store_background_raw_ir_values();
        wait(0.05);
        sensors.store_illuminated_raw_ir_values();
        pc.printf("Sample %d     1: %04d-%04d  2: %04d-%04d  3: %04d-%04d  4: %04d-%04d  5: %04d-%04d  6: %04d-%04d  7: %04d-%04d  8: %04d-%04d\n", (test_substep+1),
                  sensors.get_background_raw_ir_value(0),          sensors.get_illuminated_raw_ir_value(0),
                  sensors.get_background_raw_ir_value(1),          sensors.get_illuminated_raw_ir_value(1),
                  sensors.get_background_raw_ir_value(2),          sensors.get_illuminated_raw_ir_value(2),
                  sensors.get_background_raw_ir_value(3),          sensors.get_illuminated_raw_ir_value(3),
                  sensors.get_background_raw_ir_value(4),          sensors.get_illuminated_raw_ir_value(4),
                  sensors.get_background_raw_ir_value(5),          sensors.get_illuminated_raw_ir_value(5),
                  sensors.get_background_raw_ir_value(6),          sensors.get_illuminated_raw_ir_value(6),
                  sensors.get_background_raw_ir_value(7),          sensors.get_illuminated_raw_ir_value(7));
        sprintf(test_message,"%d:%4d-%4d",test_substep+1,sensors.get_background_raw_ir_value(test_substep),sensors.get_illuminated_raw_ir_value(test_substep));
        wait_us(SELF_TEST_PERIOD);
    }
    if(test_step == 0) {
        // Power self-test
        switch(test_substep) {
            case 0: { // Battery Voltage
                float battery_voltages [SAMPLE_SIZE];
                float mean_battery_voltage = 0;
                float sd_battery_voltage = 0;
                for(int i=0; i<SAMPLE_SIZE; i++) {
                    battery_voltages[i]=sensors.get_battery_voltage ();
                    mean_battery_voltage += battery_voltages[i];
                    wait_us(SELF_TEST_PERIOD);
                }
                mean_battery_voltage /= SAMPLE_SIZE;
                for(int i=0; i<SAMPLE_SIZE; ++i) sd_battery_voltage += pow(battery_voltages[i] - mean_battery_voltage, 2);
                sd_battery_voltage = sqrt(sd_battery_voltage/SAMPLE_SIZE);

                sprintf(test_message,"BATTERY: %1.3fV",mean_battery_voltage);
                pc.printf(" - Battery Voltage    : %1.4fV [SD = % 1.4f]\n",mean_battery_voltage,sd_battery_voltage);
                if(mean_battery_voltage < 3.6) {
                    pc.printf(" - WARNING            : Battery voltage low\n");
                    test_warnings++;
                }
                break;
            }
            case 1: {// DC Voltage
                float dc_voltages [SAMPLE_SIZE];
                float mean_dc_voltage = 0;
                float sd_dc_voltage = 0;
                for(int i=0; i<SAMPLE_SIZE; i++) {
                    dc_voltages[i]=sensors.get_dc_voltage ();
                    mean_dc_voltage += dc_voltages[i];
                    wait_us(SELF_TEST_PERIOD);
                }
                mean_dc_voltage /= SAMPLE_SIZE;
                for(int i=0; i<SAMPLE_SIZE; ++i) sd_dc_voltage += pow(dc_voltages[i] - mean_dc_voltage, 2);
                sd_dc_voltage = sqrt(sd_dc_voltage/SAMPLE_SIZE);

                sprintf(test_message,"DC     : %1.3fV",mean_dc_voltage);
                pc.printf(" - DC Input Voltage   : %1.4fV [SD = % 1.4f]\n",mean_dc_voltage,sd_dc_voltage);
                if(mean_dc_voltage < 0.5) {
                    pc.printf(" - WARNING            : DC voltage low - check charging resistor\n");
                    test_warnings++;
                }
                break;
            }


            case 2: { // Current
                float currents [SAMPLE_SIZE];
                float mean_current = 0;
                float sd_current = 0;
                for(int i=0; i<SAMPLE_SIZE; i++) {
                    currents[i]=sensors.get_current ();
                    mean_current += currents[i];
                    wait_us(SELF_TEST_PERIOD);
                }
                mean_current /= SAMPLE_SIZE;
                for(int i=0; i<SAMPLE_SIZE; ++i) sd_current += pow(currents[i] - mean_current, 2);
                sd_current = sqrt(sd_current/SAMPLE_SIZE);

                sprintf(test_message,"CURRENT: %1.3fV",mean_current);
                pc.printf(" - Load Current       : %1.4fA [SD = % 1.4f]\n",mean_current,sd_current);
                if(mean_current > 0.2) {
                    pc.printf(" - WARNING            : Higher than expected load current\n");
                    test_warnings++;
                }
                break;
            }

            case 3: {// System temperature
                float temps[SAMPLE_SIZE];
                float mean_temp = 0;
                float sd_temp = 0;
                for(int i=0; i<SAMPLE_SIZE; i++) {
                    temps[i]=sensors.get_temperature();
                    mean_temp += temps[i];
                    wait_us(SELF_TEST_PERIOD);
                }
                mean_temp /= SAMPLE_SIZE;
                for(int i=0; i<SAMPLE_SIZE; ++i) sd_temp += pow(temps[i] - mean_temp, 2);
                sd_temp = sqrt(sd_temp/SAMPLE_SIZE);

                sprintf(test_message,"TEMP   : %3.2fC",mean_temp);
                pc.printf(" - System Temperature : %3.2fC [SD = % 1.4f]\n",mean_temp,sd_temp);
                if(mean_temp > 32) {
                    pc.printf(" - WARNING            : High system temperature detected\n");
                    test_warnings++;
                }
                if(mean_temp < 10) {
                    pc.printf(" - WARNING            : Low system temperature detected\n");
                    test_warnings++;
                }
                break;
            }
        }
    }

    display.set_position(1,0);
    display.write_string("                ");
    display.set_position(1,0);
    display.write_string(test_message);
    test_substep++;
    if(test_substep >= next_step[test_step]) {
        test_substep = 0;
        test_step ++;
        if(test_step == 4) test_step = 0;
    }

    demo_timeout.attach_us(this,&Demo::quick_test_cycle, wait_period);
}

void Demo::toggle_quick_test()
{
    quick_test = 1 - quick_test;
    if(quick_test == 0) {
        pc.printf("________________________________________\n");
        pc.printf("Self test stopped at %f with %d warnings\n\n",psi.get_uptime(),test_warnings);
        demo_running = 0;
        demo_timeout.detach();
        display.set_backlight_brightness(1);
        display.clear_display();
        display.write_string("---QUICK TEST---");


    } else {
        // Reset all LEDs, motors, display
        pc.printf("\n________________________________________\n");
        pc.printf("Self test started at %f\n\n",psi.get_uptime(),test_warnings);
        display.clear_display();
        test_step = 0;
        test_substep = 0;
        demo_running = 1;
        demo_timeout.attach_us(this,&Demo::quick_test_cycle,1000);
    }
}

void Demo::start_line_demo()
{
    display.set_backlight_brightness(0);
    time_out = 0.25f;
    demo_timer.start();
    state = 0;
    speed = 0;
    led_step = 0;
    demo_running = 1;
    demo_timeout.attach_us(this,&Demo::line_demo_cycle,1000);
}

void Demo::start_obstacle_demo()
{
    display.set_backlight_brightness(0);
    time_out = 0.25f;
    demo_timer.start();
    state = 0;
    speed = 0;
    led_step = 0;
    demo_running = 1;
    demo_timeout.attach_us(this,&Demo::obstacle_demo_cycle,1000);
}

void Demo::start_colour_demo()
{
    display.set_backlight_brightness(0);
    time_out = 0.25f;
    demo_timer.start();
    state = 0;
    speed = 0;
    led_step = 0;
    demo_running = 1;
    colour.start_colour_ticker(100);
    demo_timeout.attach_us(this,&Demo::colour_demo_cycle,1000);
}

void Demo::start_stress_demo()
{
    display.set_backlight_brightness(0.25);
    display.write_string("STRESS TEST");
    display.set_position(1,0);
    display.write_string("----25%----");
    time_out = 0.04f;
    demo_timer.start();
    state = 0;
    speed = 0;
    stress_step = 0;
    spin_step = 0;
    demo_running = 1;
    demo_timeout.attach_us(this,&Demo::stress_demo_cycle,1000);
}

void Demo::start_spinning_demo()
{
    display.set_backlight_brightness(0);
    time_out = 0.0f;
    demo_timer.start();
    state = 0;
    speed = 0;
    led_step = 0;
    spin_step = 0;
    demo_running = 1;
    demo_timeout.attach_us(this,&Demo::spinning_demo_cycle,1000);
}

void Demo::line_demo_cycle()
{
    if(demo_timer.read() > time_out) {
        sensors.store_line_position();
        if(line_found) {
            time_out = 0.01f;
            state = 0;
            // Get the position of the line.
            lf_current_pos_of_line = line_position;
            lf_proportional = lf_current_pos_of_line;

            // Compute the derivative
            lf_derivative = lf_current_pos_of_line - lf_previous_pos_of_line;

            // Compute the integral
            lf_integral += lf_proportional;

            // Remember the last position.
            lf_previous_pos_of_line = lf_current_pos_of_line;

            // Compute the power
            lf_power = (lf_proportional * (LF_P_TERM) ) + (lf_integral*(LF_I_TERM)) + (lf_derivative*(LF_D_TERM)) ;

            // Compute new speeds
            lf_right = lf_speed-lf_power;
            lf_left  = lf_speed+lf_power;

            // limit checks
            if (lf_right < 0)
                lf_right = 0;
            else if (lf_right > 1.0f)
                lf_right = 1.0f;

            if (lf_left < 0)
                lf_left = 0;
            else if (lf_left > 1.0f)
                lf_left = 1.0f;
        } else {
            //Cannot see line: hunt for it
            if(lf_left > lf_right) {
                //Currently turning left, keep turning left
                state ++;
                float d_step = state * 0.04;
                lf_left = 0.2 + d_step;
                lf_right = -0.2 - d_step;
                if(state > 20) {
                    state = 0;
                    lf_right = 0.2;
                    lf_left = -0.2;
                    time_out += 0.01f;
                    if(time_out > 0.1f) demo_running = 0;
                }
            } else {
                //Currently turning right, keep turning right
                state ++;
                float d_step = state * 0.04;
                lf_left = -0.2 - d_step;
                lf_right = 0.2 + d_step;
                if(state > 20) {
                    state = 0;
                    lf_right = -0.2;
                    lf_left = 0.2;
                    time_out += 0.01f;
                    if(time_out > 0.1f) demo_running = 0;
                }
            }
        }
        // set speed
        motors.set_left_motor_speed(lf_left);
        motors.set_right_motor_speed(lf_right);


        demo_timer.reset();
    }
    if(demo_running == 1)demo_timeout.attach_us(this,&Demo::line_demo_cycle,500);
    else {
        motors.stop();
        display.set_backlight_brightness(bl_brightness * 0.01f);
    }
}

void Demo::stress_demo_cycle()
{
    if(demo_timer.read() > time_out) {
        float pct = 0.25 + (0.25 * stress_step);
        switch(state) {
            case 0:
                if(spin_step % 2 == 0) {
                    motors.forward(pct);
                    led.set_leds(0xFF,0xFF);
                } else {
                    motors.backward(pct);
                    led.set_leds(0,0xFF);
                }
                spin_step ++;
                if(spin_step > 199) {
                    state ++;
                    spin_step = 0;
                }
                break;
            case 1:
                if(stress_step < 3) stress_step ++;
                float pct = 0.25 + (0.25 * stress_step);
                display.set_backlight_brightness(pct);
                display.set_position(1,0);
                switch(stress_step) {
                    case 1:
                        display.write_string("----50%----");
                        break;
                    case 2:
                        display.write_string("----75%----");
                        break;
                    case 3:
                        display.write_string("---100%----");
                        break;
                }
                state = 0;
                break;
        }
        demo_timer.reset();
    }
    if(demo_running == 1)demo_timeout.attach_us(this,&Demo::stress_demo_cycle,500);
    else {
        motors.stop();
        display.set_backlight_brightness(bl_brightness * 0.01f);
    }
}

void Demo::spinning_demo_cycle()
{
    if(demo_timer.read() > time_out) {
        switch(state) {
            case 0: //Robot is stopped
                led.set_leds(0,0xFF);
                led.set_center_led(1,1);
                speed = 0.1f;
                motors.brake();
                time_out = 0.5;
                state = 1;
                led_step = 0;
                break;
            case 1: //Motor is turning right, accelerating
                time_out = 0.1;
                led.set_center_led(2,1);
                switch(led_step) {
                    case 0:
                        led.set_leds(0x01,0);
                        break;
                    case 1:
                        led.set_leds(0x02,0);
                        break;
                    case 2:
                        led.set_leds(0x04,0);
                        break;
                    case 3:
                        led.set_leds(0x08,0);
                        break;
                    case 4:
                        led.set_leds(0x10,0);
                        break;
                    case 5:
                        led.set_leds(0x20,0);
                        break;
                    case 6:
                        led.set_leds(0x40,0);
                        break;
                    case 7:
                        led.set_leds(0x80,0);
                        break;
                }
                led_step ++;
                if(led_step == 8) led_step =0;
                if(speed < 1) {
                    speed += 0.0125;
                    motors.turn(speed);
                } else {
                    state = 2;
                    spin_step = 0;
                    led_step =0;
                }
                break;
            case 2: //Motor is turning right, full speed
                led.set_center_led(3,1);
                switch(led_step) {
                    case 0:
                        led.set_leds(0x33,0x33);
                        break;
                    case 1:
                        led.set_leds(0x66,0x66);
                        break;
                    case 2:
                        led.set_leds(0xCC,0xCC);
                        break;
                    case 3:
                        led.set_leds(0x99,0x99);
                        break;
                }
                led_step ++;
                if(led_step == 4) led_step = 0;
                spin_step ++;
                if(spin_step == 40) {
                    state = 3;
                    led_step = 0;
                }
                break;
            case 3: //Motor is turning right, decelerating
                led.set_center_led(2,1);
                switch(led_step) {
                    case 0:
                        led.set_leds(0x01,0);
                        break;
                    case 1:
                        led.set_leds(0x02,0);
                        break;
                    case 2:
                        led.set_leds(0x04,0);
                        break;
                    case 3:
                        led.set_leds(0x08,0);
                        break;
                    case 4:
                        led.set_leds(0x10,0);
                        break;
                    case 5:
                        led.set_leds(0x20,0);
                        break;
                    case 6:
                        led.set_leds(0x40,0);
                        break;
                    case 7:
                        led.set_leds(0x80,0);
                        break;
                }
                if(led_step == 0) led_step =8;
                led_step --;
                if(speed > 0.1) {
                    speed -= 0.025;
                    motors.turn(speed);
                } else {
                    state = 4;
                    spin_step = 0;
                    led_step =0;
                }
                break;
            case 4: //Robot is stopped
                led.set_leds(0,0xFF);
                led.set_center_led(1,1);
                speed = 0.1f;
                motors.brake();
                time_out = 0.5;
                led_step =0;
                state = 5;
                break;
            case 5: //Motor is turning left, accelerating
                time_out = 0.1;
                led.set_center_led(2,1);
                switch(led_step) {
                    case 0:
                        led.set_leds(0x01,0);
                        break;
                    case 1:
                        led.set_leds(0x02,0);
                        break;
                    case 2:
                        led.set_leds(0x04,0);
                        break;
                    case 3:
                        led.set_leds(0x08,0);
                        break;
                    case 4:
                        led.set_leds(0x10,0);
                        break;
                    case 5:
                        led.set_leds(0x20,0);
                        break;
                    case 6:
                        led.set_leds(0x40,0);
                        break;
                    case 7:
                        led.set_leds(0x80,0);
                        break;
                }
                if(led_step == 0) led_step =8;
                led_step --;
                if(speed < 1) {
                    speed += 0.0125;
                    motors.turn(-speed);
                } else {
                    state = 6;
                    spin_step = 0;
                    led_step = 0;
                }
                break;
            case 6: //Motor is turning left, full speed
                led.set_center_led(3,1);
                switch(led_step) {
                    case 0:
                        led.set_leds(0x33,0x33);
                        break;
                    case 1:
                        led.set_leds(0x66,0x66);
                        break;
                    case 2:
                        led.set_leds(0xCC,0xCC);
                        break;
                    case 3:
                        led.set_leds(0x99,0x99);
                        break;
                }
                if(led_step == 0) led_step = 4;
                led_step --;
                spin_step ++;
                if(spin_step == 40) {
                    state = 7;
                    led_step = 0;
                }
                break;
            case 7: //Motor is turning left, decelerating
                led.set_center_led(2,1);
                switch(led_step) {
                    case 0:
                        led.set_leds(0x01,0);
                        break;
                    case 1:
                        led.set_leds(0x02,0);
                        break;
                    case 2:
                        led.set_leds(0x04,0);
                        break;
                    case 3:
                        led.set_leds(0x08,0);
                        break;
                    case 4:
                        led.set_leds(0x10,0);
                        break;
                    case 5:
                        led.set_leds(0x20,0);
                        break;
                    case 6:
                        led.set_leds(0x40,0);
                        break;
                    case 7:
                        led.set_leds(0x80,0);
                        break;
                }
                led_step ++;
                if(led_step == 8) led_step =0;
                if(speed > 0.1) {
                    speed -= 0.025;
                    motors.turn(-speed);
                } else {
                    state = 0;
                    spin_step = 0;
                    led_step = 0;
                }
                break;
        }
        demo_timer.reset();
    }
    if(demo_running == 1)demo_timeout.attach_us(this,&Demo::spinning_demo_cycle,500);
    else {
        motors.stop();
        display.set_backlight_brightness(bl_brightness * 0.01f);
    }
}

void Demo::colour_demo_cycle()
{
    if(demo_timer.read() > time_out) {
        int col = colour.get_detected_colour();
        switch(col) {
            case 0:
                led.set_leds(0,0xFF);
                led.set_center_led(1,1);
                break;
            case 1:
                led.set_leds(0xFF,0xFF);
                led.set_center_led(3,1);
                break;
            case 2:
                led.set_leds(0xFF,0);
                led.set_center_led(2,1);
                break;
            default:
                led.set_leds(0,0);
                led.set_center_led(0,0);
                break;
        }
        switch(state) {
            case 0: //Robot is stopped
                speed = 0.2f;
                motors.forward(speed);
                time_out = 0.05;
                state = 1;
                break;
            case 1: { //Motor is moving forward
                sensors.store_ir_values();
                int front_right = sensors.read_illuminated_raw_ir_value(0);
                int front_left = sensors.read_illuminated_raw_ir_value(7);
                if(front_left > 400 || front_right > 400) {
                    motors.brake();
                    time_out = 0.04;
                    if(front_left > front_right)state=2;
                    else state=3;
                } else {
                    if(speed < 0.5) {
                        speed += 0.03;
                        motors.forward(speed);
                    }
                }
                break;
            }
            case 2: //Turn right
                motors.set_left_motor_speed(0.35);
                motors.set_right_motor_speed(-0.35);
                time_out = 0.5;
                state = 0;
                break;
            case 3: //Turn left
                motors.set_left_motor_speed(-0.35);
                motors.set_right_motor_speed(0.35);
                time_out = 0.5;
                state = 0;
                break;
        }
        demo_timer.reset();
    }
    if(demo_running == 1)demo_timeout.attach_us(this,&Demo::colour_demo_cycle,200);
    else {
        motors.stop();
        display.set_backlight_brightness(bl_brightness * 0.01f);
    }
}


void Demo::obstacle_demo_cycle()
{

    if(demo_timer.read() > time_out) {
        switch(state) {
            case 0: //Robot is stopped
                led.set_leds(0,0xFF);
                led.set_center_led(1,0.4);
                speed = 0.3f;
                motors.forward(speed);
                time_out = 0.05;
                state = 1;
                break;
            case 1: { //Motor is moving forward
                sensors.store_ir_values();
                int front_right = sensors.read_illuminated_raw_ir_value(0);
                int front_left = sensors.read_illuminated_raw_ir_value(7);
                if(front_left > 400 || front_right > 400) {
                    motors.brake();
                    time_out = 0.04;
                    if(front_left > front_right)state=2;
                    else state=3;
                } else {
                    if(speed < 0.5) {
                        speed += 0.03;
                        motors.forward(speed);
                    }
                    switch(led_step) {
                        case 0:
                            led.set_leds(0x01,0);
                            break;
                        case 1:
                            led.set_leds(0x38,0);
                            break;
                        case 2:
                            led.set_leds(0x6C,0);
                            break;
                        case 3:
                            led.set_leds(0xC6,0);
                            break;
                        case 4:
                            led.set_leds(0x83,0);
                            break;
                    }
                    led.set_center_led(2, 0.6);
                    led_step ++;
                    if(led_step == 5) led_step = 0;
                }
                break;
            }
            case 2: //Turn right
                motors.set_left_motor_speed(0.85);
                motors.set_right_motor_speed(-0.85);
                time_out = 0.4;
                state = 0;
                led.set_leds(0x0E,0x0E);
                led.set_center_led(3,0.5);
                break;
            case 3: //Turn left
                motors.set_left_motor_speed(-0.85);
                motors.set_right_motor_speed(0.85);
                time_out = 0.4;
                state = 0;
                led.set_leds(0xE0,0xE0);
                led.set_center_led(3,0.5);
                break;
        }
        demo_timer.reset();
    }
    if(demo_running == 1)demo_timeout.attach_us(this,&Demo::obstacle_demo_cycle,200);
    else {
        motors.stop();
        display.set_backlight_brightness(bl_brightness * 0.01f);
    }
}


void Demo::demo_update_leds()
{
    char red = 0;
    char green = 0;
    for(int i=0; i<8; i++) {
        if(led_state[i]==1 || led_state[i]==3) red+=(1<<i);
        if(led_state[i]==2 || led_state[i]==3) green+=(1<<i);
    }
    led.set_leds(green,red);
    float brightness_f = brightness / 100.0f;
    led.set_center_led(led_state[8], brightness_f);
    led.set_base_led(base_led_state);
}