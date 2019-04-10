/* University of York Robotics Laboratory PsiSwarm Library: LED Functions Header File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: led.h
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


#ifndef LED_H
#define LED_H

/**
 * Led class
 * Functions to control the various LEDs on the robot
 *
 * Example:
 * @code
 * #include "psiswarm.h"
 *
 * int main() {
 *     init();
 *     led.set_led(0,1);       //Set the outer LED number 0 (North) to red
 *     led.set_led(4,3);       //Set the outer LED number 4 (South) to orange (red+green)
 * }
 * @endcode
*/
class Led
{

public:

    /** Set all 8 outer LEDs to the defined colour sequence
    * @param green - An 8-bit description of the green leds eg(0b00000001) means that LED 7 green is on, rest are off
    * @param red - An 8-bit description of the red   leds eg(0b11111110) means that LED 7 red is off, rest are on
    */
    void set_leds(char green, char red);

    /** Set the green component of all 8 outer LEDs to the defined colour sequence
    * @param green - An 8-bit description of the green leds eg(0b00000001) means that LED 7 green is on, rest are off
    */
    void set_green_leds(char green);

    /** Set the red component of all 8 outer LEDs to the defined colour sequence
    * @param red - An 8-bit description of the red   leds eg(0b11111110) means that LED 7 red is off, rest are on
    */
    void set_red_leds(char red);

    /** Set the state of an invididual outer LED without affecting other LEDs
    * @param led - The LED to change state of (range 0 to 7)
    * @param state - 0 for off, 1 for red, 2 for green, 3 for orange
    */
    void set_led(char led, char state);

    /** Set the state of the base LEDs [if fitted]
    * @param state - 0 for off, 1 for on
    */
    void set_base_led(char state);

    /** Turns on all outer LEDs for a period of time defined by timeout then restore their previous state
    * @param timeout - The time (in seconds) to keep LEDs on
    */
    void blink_leds(float timeout);

    /** Set the state the center LED
    * @param state - 0 for off, 1 for red, 2 for green, 3 for orange
    */
    void set_center_led(char state);

    /** Set the state the center LED with brightness control
    * @param state - 0 for off, 1 for red, 2 for green, 3 for orange
    * @param brightness - brightness of LED [PWM duty cycle] - range 0.0 to 1.0
    */
    void set_center_led(char state, float brightness);

    /** Set the brightness of center LED without changing state
    * @param brightness - brightness of LED [PWM duty cycle] - range 0.0 to 1.0
    */
    void set_center_led_brightness(float brightness);

    /** Returns the current state of the outer LEDs
    * @return A 16-bit value when MSB represent the green states and LSB the red states of the 8 LEDs
    */
    unsigned short get_led_states(void);

    /** Store the current LED states for use with restore_led_states()
    */
    void save_led_states(void);

    /** Restore the LED states to those set usign store_led_states()
    */
    void restore_led_states(void);

    void IF_init_leds(void);
    void IF_update_leds(void);

};
#endif