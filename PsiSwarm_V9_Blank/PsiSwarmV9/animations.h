/* University of York Robotics Laboratory PsiSwarm Library: Animations Header File
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * Library of simple predetermined movements
 *
 * File: animations.h
 * [Was dances.h in version 0.7]
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

#ifndef ANIMATIONS_H
#define ANIMATIONS_H

/**
 *  The Animations class contains simple predefined LED animations and dances
*/
class Animations{
    public:
    /**
     * Make the robot vibrate (turn rapidly left & right) for approximately 1 second with LED flashes; restores LED states after action
    */
    void vibrate(void);
    
    /**
     * Patterns LEDs from back to front of robot 3 times then blinks at the front; animation takes about 1 second; restores LED states after action
      */
    void led_run1(void);
    
    /** 
     * Sets the colour for single-colour LED animations (default = 1)
     * @param colour The colour LED to use in the animation (1 = red, 2 = green, 3 = orange)
     */
    void set_colour(char colour);

    private:
    void IF_vibrate(void);
    void IF_led_run1(void);
    char animation_running;
    char hold_colour;
    char animation_counter;
    Timeout animation_timeout;
};

#endif