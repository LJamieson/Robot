/* University of York Robotics Laboratory PsiSwarm Library: Animations Source File
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * Library of simple predetermined movements and LED animations
 *
 * File: animations.cpp
 * [Was dances.cpp in version 0.7]
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

char hold_colour = 1;
char animation_running = 0;
//
void Animations::set_colour(char colour)
{
    hold_colour = colour;
}

//Blink through LED pattern
void Animations::led_run1(void)
{
    if(animation_running == 0){
        animation_counter = 0;
        animation_running = 1;
        IF_led_run1();   
    }else{
        psi.debug("WARNING: Animation called before previous animation finished, ignoring\n");   
    }
}

void Animations::IF_led_run1()
{
    if(animation_counter == 0)led.save_led_states();
    char led_pattern = 0x0;
    if(animation_counter < 16){
        switch(animation_counter % 4){
            case 0:
                led_pattern = 0x10;
            break;
            case 1:
                led_pattern = 0x28;
            break;
            case 2:
                led_pattern = 0x44;
            break;
            case 3:
                led_pattern = 0x83;
            break;   
        }   
    }
    if(animation_counter == 16 || animation_counter == 18 || animation_counter == 20) led_pattern = 0x01;
    char green_state = 0;
    char red_state = 0;
    if(hold_colour % 2 == 1) red_state = led_pattern;
    if(hold_colour > 1) green_state = led_pattern;
    led.set_leds(green_state,red_state);
    animation_counter++;
    if(animation_counter < 21) {
        animation_timeout.attach(this, &Animations::IF_led_run1, 0.05f);
    } else {
        animation_counter = 0;
        led.restore_led_states();
        animation_running = 0;
    }
}

//Do a simple wiggle
void Animations::vibrate(void)
{
    if(animation_running == 0){
        animation_counter = 0;
        animation_running = 1;
        IF_vibrate();   
    }else{
        psi.debug("WARNING: Animation called before previous animation finished, ignoring\n");   
    }
}

void Animations::IF_vibrate(void)
{
    if(animation_counter == 0)led.save_led_states();
    if(animation_counter % 2 == 0) {
        led.set_leds(0xC7,0x00);
        motors.turn(1.0);
    } else {
        led.set_leds(0x00,0xC7);
        motors.turn(-1.0);
    }
    animation_counter++;

    if(animation_counter < 14) {
        float wiggle_timeout_period = 0.06;
        //Move less on first 'wiggle' so that we stay in roughly the same place!
        if(animation_counter == 0) wiggle_timeout_period = 0.03;
        animation_timeout.attach(this, &Animations::IF_vibrate, wiggle_timeout_period);
    } else {
        animation_counter = 0;
        motors.brake();
        led.restore_led_states();
        animation_running = 0;
    }
}