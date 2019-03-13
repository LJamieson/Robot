/* University of York Robotics Laboratory PsiSwarm Library: LED Functions Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: led.cpp
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

char green_led_states;
char red_led_states;
char center_led_state;

char held_red_states;
char held_green_states;
Timeout blink_led_timeout;

unsigned short Led::get_led_states()
{
    return (green_led_states << 8) + red_led_states;
}

void Led::set_leds(char green, char red)
{
    green_led_states = green;
    red_led_states = red;
    IF_update_leds();
}

void Led::set_base_led(char state)
{
    i2c_setup.IF_set_base_LED(state);
}

void Led::set_green_leds(char green)
{
    green_led_states = green;
    IF_update_leds();
}

void Led::set_red_leds(char red)
{
    red_led_states = red;
    IF_update_leds();
}

void Led::set_led(char led, char state)
{
    if(state % 2 == 1) red_led_states |= 1 << led;
    else red_led_states &= ~(1 << led);
    if(state / 2) green_led_states |= 1 << led;
    else green_led_states &= ~(1 << led);
    IF_update_leds();
}

void Led::blink_leds(float timeout)
{
    save_led_states();
    set_leds(0xFF,0xFF);
    blink_led_timeout.attach(this,&Led::restore_led_states, timeout);
}

void Led::set_center_led(char state)
{
    set_center_led(state, center_led_brightness);
}

void Led::set_center_led(char state, float brightness)
{
    center_led_brightness = brightness;
    center_led_state = state;
    switch(center_led_state) {
        case 0:
            center_led_red.write(0);
            center_led_green.write(0);
            break;
        case 1:
            center_led_red.write(center_led_brightness / 4);
            center_led_green.write(0);
            break;
        case 2:
            center_led_red.write(0);
            center_led_green.write(center_led_brightness);
            break;
        case 3:
            center_led_red.write(center_led_brightness / 4);
            center_led_green.write(center_led_brightness);
            break;
    }
}

void Led::set_center_led_brightness(float brightness)
{
    set_center_led(center_led_state,brightness);
}

void Led::save_led_states()
{
    held_red_states = red_led_states;
    held_green_states = green_led_states;
}

void Led::restore_led_states()
{
    set_leds(held_green_states,held_red_states);
}

void Led::IF_init_leds()
{
    green_led_states = 0;
    red_led_states = 0;
    center_led_red.period_us(100);
    center_led_green.period_us(100);
    set_center_led(0,0.2);
}

void Led::IF_update_leds()
{
    char led_byte_0 =  (((green_led_states & (1 << 3)) == 0) << 7) +
                       (((green_led_states & (1 << 2)) == 0) << 5) +
                       (((green_led_states & (1 << 1)) == 0) << 3) +
                       (((green_led_states & (1)) == 0) << 1) +
                       (((red_led_states & (1 << 3)) == 0) << 6) +
                       (((red_led_states & (1 << 2)) == 0) << 4) +
                       (((red_led_states & (1 << 1)) == 0) << 2) +
                       (((red_led_states & (1)) == 0));
    char led_byte_1 =  (((green_led_states & (1 << 7)) == 0) << 7) +
                       (((green_led_states & (1 << 6)) == 0) << 5) +
                       (((green_led_states & (1 << 5)) == 0) << 3) +
                       (((green_led_states & (1 << 4)) == 0) << 1) +
                       (((red_led_states & (1 << 7)) == 0) << 6) +
                       (((red_led_states & (1 << 6)) == 0) << 4) +
                       (((red_led_states & (1 << 5)) == 0) << 2) +
                       (((red_led_states & (1 << 4)) == 0));
    i2c_setup.IF_write_to_led_ic(led_byte_0,led_byte_1);
}