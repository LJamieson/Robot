/* University of York Robotics Laboratory PsiSwarm Library: PsiSwarm C++ Core Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: psiswarm.cpp
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

//Setup class instances
Display display;  //Connects to i2c(p28,p27), reset(p29), backlight(p30)
Motors motors;
Eprom eprom;
Led led;
Sensors sensors;
SerialControl serial;
Sound sound;
Setup i2c_setup;
Demo demo;
Animations animations;
Basic basic;
Colour colour;

//Setup MBED connections to PsiSwarm Robot
Serial pc(USBTX,USBRX);
I2C primary_i2c (p9, p10);
InterruptIn gpio_interrupt (p12);
Serial bt(p13, p14);
AnalogIn vin_current(p15);
AnalogIn vin_battery(p16);
AnalogIn vin_dc(p17);
PwmOut motor_left_f (p21);
PwmOut motor_left_r (p22);
PwmOut motor_right_f(p23);
PwmOut motor_right_r(p24);
PwmOut center_led_red(p25);
PwmOut center_led_green(p26);
DigitalOut mbed_led1(LED1);
DigitalOut mbed_led2(LED2);
DigitalOut mbed_led3(LED3);
DigitalOut mbed_led4(LED4);

float center_led_brightness;
float backlight_brightness;

Ticker event_handler;
Timer uptime;
Timeout pause_usercode_timeout;
Ticker ultrasonic_ticker;
Timeout ultrasonic_timeout;
int timer_minute_count;
Ticker timer_ticker;

float firmware_version;
float pcb_version;
float serial_number;

char has_compass=0;
char has_side_ir=1;
char has_base_ir=1;
char has_base_colour_sensor=0;
char has_top_colour_sensor=0;
char has_wheel_encoders=0;
char has_audio_pic=0;
char has_ultrasonic_sensor=0;
char has_temperature_sensor=0;
char has_recharging_circuit=0;
char has_433_radio=0;

char robot_id;
char previous_robot_id;

char wheel_encoder_byte;
char previous_wheel_encoder_byte;
signed int left_encoder;
signed int right_encoder;

char time_based_motor_action = 0;

char testing_voltage_regulators_flag = 1;
char power_good_motor_left = 2;
char power_good_motor_right = 2;
char power_good_infrared = 2;
char status_dc_in = 2;
char status_charging = 2;

char switch_byte;
char previous_switch_byte;

char use_motor_calibration = USE_MOTOR_CALIBRATION;
char motor_calibration_set;
char base_ir_calibration_set;
char base_colour_calibration_set;
float left_motor_calibration_value = 1.0;
float right_motor_calibration_value = 1.0;
float left_motor_stall_offset = 0.0;
float right_motor_stall_offset = 0.0;

char debug_mode = DEBUG_MODE;
char debug_output = DEBUG_OUTPUT_STREAM;

char firmware_bytes[80];

int base_colour_sensor_raw_values [4];
int top_colour_sensor_raw_values [4];

char waiting_for_ultrasonic = 0;
int ultrasonic_distance = 0;
char ultrasonic_distance_updated = 0;


float line_position = 0;
char line_found = 0;

unsigned short background_ir_values [8];
unsigned short illuminated_ir_values [8];
float reflected_ir_distances [8];
char ir_values_stored = 0;
unsigned short background_base_ir_values [5];
unsigned short illuminated_base_ir_values [5];
char base_ir_values_stored = 0;

float motor_left_speed;
float motor_right_speed;
char motor_left_brake;
char motor_right_brake;

char demo_on = 0;
char event = 0;
char change_id_event = 0;
char encoder_event = 0;
char switch_event = 0;
char user_code_running = 0;
char user_code_restore_mode = 0;
char system_warnings = 0;
short boot_count = 0;

vector<string> basic_filenames; //filenames are stored in a vector string
char psi_basic_file_count = 0;
char use_flash_basic = 0;
char file_transfer_mode = 0;

int ir_pulse_delay = DEFAULT_IR_PULSE_DELAY;
int base_ir_pulse_delay = DEFAULT_BASE_IR_PULSE_DELAY;

/**
 * init()
 *
 * Main initialisation routine for the PsiSwarm robot
 *
 * Set up the GPIO expansion ICs, launch demo mode if button is held
 */
void Psiswarm::init()
{
    firmware_version=0;
    timer_minute_count = 0;
    timer_ticker.attach(this,&Psiswarm::IF_update_minutes, 300);
    uptime.start();
    primary_i2c.frequency(400000);
    serial.setup_serial_interfaces();
    debug("\n_________________________________________________________________________\n");
    debug("--------------------PsiSwarm Robot Library %1.2f--------------------------\n",SOFTWARE_VERSION_CODE);
    debug("_________________________________________________________________________\n");
    debug("- Setting up serial interface\n");
    debug("- Set up display\n");
    display.init_display_start();
    debug("- Setting up GPIO expansion\n");
    i2c_setup.IF_setup_gpio_expansion_ic();
    debug("- Reading firmware: ");
    if(eprom.read_firmware() == 1) {
        debug("Version %3.2f\n",firmware_version);
        IF_get_hardware_description();
        if(use_motor_calibration) {
            if(!motor_calibration_set) {
                if(firmware_version < 1.1) {
                    debug("- WARNING: Firmware incompatible with motor calibration\n");
                    debug("- WARNING: Please update the firmware to use this feature.\n");
                    use_motor_calibration = 0;
                } else {
                    debug("- WARNING: Motor calibration values have not been stored in firmware\n");
                    debug("- WARNING: Run motor calibration routine to use this feature.\n");
                    use_motor_calibration = 0;
                }
            } else {
                debug("- Motor calibration in use\n- [LEFT:%0.4f (%1.2f OFFSET) RIGHT:%0.4f (%2.3f OFFSET)]\n",left_motor_calibration_value,left_motor_stall_offset,right_motor_calibration_value,right_motor_stall_offset);
            }
        }
        if(base_ir_calibration_set != 1) {
            if(firmware_version < 1.2) {
                debug("- WARNING: Firmware incompatible with base IR sensor calibration\n");
                debug("- WARNING: Please update the firmware to use this feature.\n");
            }  else {
                debug("- WARNING: Base IR calibration values not stored in firmware\n");
                debug("- WARNING: Run sensor calibration routine to use this feature.\n");
            }
            // Set default calibration values for base IR sensor
            sensors.IF_set_base_calibration_values(BIR1W,BIR2W,BIR3W,BIR4W,BIR5W,BIR1B,BIR2B,BIR3B,BIR4B,BIR5B);
        } else {
            debug("- Using base IR calibration values stored in firmware\n- %s\n",sensors.IF_get_base_calibration_values_string());
        }
        if(has_base_colour_sensor == 1 && base_colour_calibration_set != 1){
            if(firmware_version < 1.2) {
                debug("- WARNING: This firmware is incompatible with base colour sensor calibration\n");
                debug("- WARNING: Please update the firmware to use this feature.\n");
            }  else {
                debug("- WARNING: Base colour sensor calibration values not stored in firmware\n");
                debug("- WARNING: Run sensor calibration routine to use this feature.\n");
            }
            // Set default calibration values for base colour sensor
            
            colour.set_calibration_values(CS_C_BLACK,CS_R_BLACK,CS_G_BLACK,CS_B_BLACK,CS_C_WHITE,CS_R_WHITE,CS_G_WHITE,CS_B_WHITE);
        } else {
            debug("- Using base colour sensor calibration values stored in firmware\n- %s\n",colour.IF_get_calibration_values_string());
            }
        if(firmware_version < TARGET_FIRMWARE_VERSION && AUTO_UPDATE_FIRMWARE == 1) eprom.update_firmware();
        if(firmware_version > 1.1) debug("- Boot Count: %d\n",boot_count);
    } else {
        debug("INVALID\n");
        if(ENABLE_FIRMWARE_WRITER){
            debug("- Starting firmware writer\n");
            eprom.firmware_writer();
        }else{
            debug("- WARNING: Enable firmware writer in settings or use firmware writer program to set firmware\n");   
        }
    }



    if(ENABLE_BASIC == 1) {
        basic.read_list_of_file_names();
        if(psi_basic_file_count == 0) {
            debug("- No PsiBasic files found\n");
        } else use_flash_basic = 1;
    }
    debug("- Setting up PIC microcontroller\n");
    // IF_check_pic_firmware();
    debug("- Setting up LED drivers\n");
    led.IF_init_leds();
    if(i2c_setup.IF_setup_led_expansion_ic() != 0) {
        debug("- WARNING: No I2C acknowledge for LED driver\n");
        system_warnings += 1;
    }
    debug("- Setting up motor drivers\n");
    motors.init_motors();
    reset_encoders();
    if(has_temperature_sensor) {
        debug("- Setting up temperature sensor\n");
        i2c_setup.IF_setup_temperature_sensor();
    }
    if(has_base_colour_sensor) {
        debug("- Setting up base colour sensor\n");
        if(colour.IF_check_base_colour_sensor() == 1) {
            colour.colour_sensor_init();
        } else debug("- WARNING: Invalid response from colour sensor");
    }
    if(has_ultrasonic_sensor) {
        debug("- Setting up ultrasonic sensor\n");
        //enable_ultrasonic_ticker();
    }

    debug("- Robot ID: %d\n",robot_id);
    char switchstate = i2c_setup.IF_get_switch_state();
    debug("- Switch State   : %d\n",switchstate);
    debug("- Battery Voltage: %1.3fV\n",sensors.get_battery_voltage());
    debug("- DC Voltage     : %1.3fV\n",sensors.get_dc_voltage());
    debug("- Current Draw   : %1.3fA\n",sensors.get_current());
    if(has_temperature_sensor) {
        debug("- Temperature    : %1.3fC\n",sensors.get_temperature());
    }
    char demo_on = 0;
    if(ENABLE_DEMO == 1 && switchstate > 0) demo_on=1;
    display.init_display_end(demo_on);
    event_handler.attach_us(this,&Psiswarm::IF_handle_events, 1000);
    if(demo_on > 0) {
        debug("- Demo mode button is pressed\n");
        wait(0.6);
        demo_on = i2c_setup.IF_get_switch_state();
        if(demo_on > 0) demo.start_demo_mode();
        display.init_display_end(0);
    }
    debug("_________________________________________________________________________\n\n");
}

void Psiswarm::IF_update_minutes()
{
    uptime.reset();
    timer_minute_count += 5;
}

void Psiswarm::IF_handle_events()
{
    // This is the main 'operating system' thread that handles events from robot stimuli
    // By default it is run every 1ms and checks if there are events to handle
    if(event > 0) {
        // There are some events to handle.  We don't handle all events in every loop to keep the system responsive, instead they are priorised.
        if(encoder_event == 1) {
            // The encoders have changed; update the encoder values
            IF_update_encoders();
            encoder_event = 0;
            event--;
        }  else {
            if(switch_event == 1) {
                IF_update_switch();
                switch_event = 0;
                event--;
            }
            if(change_id_event == 1) {
                // The user ID for the robot has been changed
                IF_update_user_id();
                change_id_event = 0;
                event--;
            }
        }
    }
}

void Psiswarm::IF_update_encoders()
{
    char rwep = previous_wheel_encoder_byte >> 2;
    char rwe = wheel_encoder_byte >> 2;
    char lwep = previous_wheel_encoder_byte % 4;
    char lwe = wheel_encoder_byte % 4;
    //pc.printf("L:%d P:%d   R:%d P:%d  \n",lwe,lwep,rwe,rwep);
    if(lwe == 0 && lwep==1) left_encoder++;
    if(lwe == 0 && lwep==2) left_encoder--;
    if(rwe == 0 && rwep==1) right_encoder++;
    if(rwe == 0 && rwep==2) right_encoder--;
    if(left_encoder % 100 == 0) pc.printf("L:%d\n",left_encoder);
}

void Psiswarm::IF_update_user_id()
{
}

void Psiswarm::IF_update_switch()
{
    // The user switch has changed state
    // In this implementation we will only act on positive changes (rising edges)
    // Subtracting new_state from (new_state & old_state) gives the positive changes
    char positive_change = switch_byte - (switch_byte & previous_switch_byte);
    if(demo_on) demo.demo_handle_switch_event(positive_change);
    else handle_switch_event(positive_change);
}

void Psiswarm::reset_encoders()
{
    left_encoder = 0;
    right_encoder = 0;
}

void Psiswarm::debug(const char* format, ...)
{
    char buffer[256];
    if (debug_mode) {
        va_list vl;
        va_start(vl, format);
        vsprintf(buffer,format,vl);
        if(debug_output & 2) bt.printf("%s", buffer);
        if(debug_output & 1) pc.printf("%s", buffer);
        if(debug_output & 4) display.debug_page(buffer,strlen(buffer));
        va_end(vl);
    }
}

float Psiswarm::get_uptime(void)
{
    return uptime.read() + (timer_minute_count * 60);
}

void Psiswarm::pause_user_code(float period)
{
    user_code_restore_mode = user_code_running;
    user_code_running = 0;
    pause_usercode_timeout.attach(this,&Psiswarm::IF_end_pause_user_code, period);
}

void Psiswarm::IF_end_pause_user_code()
{
    user_code_running = user_code_restore_mode;
}

void Psiswarm::IF_get_hardware_description()
{
    debug("- Robot serial number %1.2f\n",serial_number);
    debug("- PCB version %1.2f\n",pcb_version);
    debug("- Hardware: ");
    if(has_compass) debug("COMPASS ");
    if(has_side_ir) debug("SIDE-IR ");
    if(has_base_ir) debug("BASE-IR ");
    if(has_base_colour_sensor) debug("BASE-COLOUR ");
    if(has_top_colour_sensor) debug("TOP-COLOUR ");
    if(has_wheel_encoders) debug("WHEEL-ENC ");
    if(has_audio_pic) debug("AUDIO ");
    if(has_ultrasonic_sensor) debug("ULTRASONIC ");
    if(has_temperature_sensor) debug("TEMPERATURE ");
    if(has_recharging_circuit) debug("RECHARGING ");
    if(has_433_radio) debug("433-RADIO");
    debug("\n");
}