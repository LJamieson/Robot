/* University of York Robotics Laboratory PsiSwarm Library: I2C Setup Header File
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: i2c_setup.h
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
 
#ifndef I2C_H
#define I2C_H

/** 
 * The Setup class contains internal functions that initiate the I2C components on the robot and send
 * the low level messages to read\write to these components.  The functions within this class are 
 * intended to be used by other classes to provide higher level functionality, so are not documented in
 * the API.
 */
class Setup
{
public:
char get_dc_status(void);

char IF_setup_led_expansion_ic(void);
void IF_setup_gpio_expansion_ic(void);

void IF_read_aux_ic_data(void);
void IF_parse_gpio_byte0(char byte);
void IF_parse_gpio_byte1(char byte);
void IF_handle_gpio_interrupt(void);
void IF_update_gpio_inputs(void);
void IF_set_base_LED(char state);
void IF_set_IR_emitter_output(char emitter, char state);
unsigned short IF_read_IR_adc_value(char adc, char index);
char IF_is_switch_pressed(void);
char IF_get_switch_state(void);
void IF_write_to_led_ic(char byte_0, char byte_1);
void IF_setup_temperature_sensor(void);
float IF_read_from_temperature_sensor(void);
};
#endif