/* University of York Robotics Laboratory PsiSwarm Library: Demo Mode Header File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: demo.h
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

#ifndef DEMO_H
#define DEMO_H


/**
 * Demo class
 * Build in demonstration\test mode for the robot that is enabled by holding the cursor switch
 * in a direction for 1 second at boot-up time.  The demonstration includes the ability to get
 * readings from most of the on-board sensors, enable LEDs and motors, and run a number of basic
 * build in demonstrations and tests, using cursor-navigated menus.
 *
 * The demo can also be enabled by calling the start_demo_mode() function.
*/
class Demo
{
public:
    /**
     * Start the demonstration mode
    */ 
    void start_demo_mode(void);
    void demo_handle_switch_event(char switch_pressed);

private:
    void demo_update_leds(void);
    void demo_event_thread(void);
    void start_line_demo(void);
    void start_obstacle_demo(void);
    void start_spinning_demo(void);
    void start_stress_demo(void);
    void start_colour_demo(void);
    void line_demo_cycle(void);
    void obstacle_demo_cycle(void);
    void spinning_demo_cycle(void);
    void stress_demo_cycle(void);
    void colour_demo_cycle(void);
    void quick_test_cycle(void);
    void toggle_quick_test(void);
};
#endif