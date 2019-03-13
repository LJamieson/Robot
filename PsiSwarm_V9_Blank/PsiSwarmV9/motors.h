/* University of York Robotics Laboratory PsiSwarm Library: Motor Functions Header File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: motors.h
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

#ifndef MOTORS_H
#define MOTORS_H

/**
 * Motors class
 * Functions to control the Psi Swarm robot motors
 *
 * Example:
 * @code
 * #include "psiswarm.h"
 *
 * int main() {
 *     init();
 *     motors.forward(0.5);    //Set the motors to forward at speed 0.5
 *     wait(0.5);
 *     motors.brake();         //Enable the hardware brake
 *     wait(0.5);
 *     motors.turn(0.5);       //Turn clockwise at 50% speed
 *     wait(0.5);
 *     motors.stop();          //Sets motor speed to zero (but not hardware brake)
 * }
 * @endcode
*/
class Motors
{
public:

    /** Set the left motor to the specified speed
    * @param speed - The set motor to the specified (range -1.0 for max. reverse to 1.0 for max. forward)
    */
    void set_left_motor_speed(float speed);

    /** Set the left motor to the specified speed
    * @param speed - The set motor to the specified (range -1.0 for max. reverse to 1.0 for max. forward)
    */
    void set_right_motor_speed(float speed);

    /** Enable the active brake on the left motor
    */
    void brake_left_motor(void);

    /** Enable the active brake on the right motor
    */
    void brake_right_motor(void);

    /** Enable the active brake on the both motors
    */
    void brake(void);

    /** Stop both motors
    * This sets the speed of both motors to 0; it does not enable the active brake
    */
    void stop(void);

    /** Sets both motors to the specified speed
    * @param speed - Set the motors to the specified speed (range -1.0 for max. reverse to 1.0 for max. forward)
    */
    void forward(float speed);

    /** Sets both motors to the specified inverted speed
    * @param speed - Set the motors to the specified speed (range -1.0 for max. forward to 1.0 for max. reverse)
    */
    void backward(float speed);

    /** Turn the robot on the spot by setting motors to equal and opposite speeds
    * @param speed - Sets the turning speed (range -1.0 for max. counter-clockwise to 1.0 for max. clockwise)
    */
    void turn(float speed);

    /** Initialise the PWM settings for the motors
    *
    */
    void init_motors(void);
    
    /** Motor calibration routine
    * Calculates stall and speed offsets for motors, stores in EEPROM
    */
    void calibrate_motors(void);

    // New time based functions (added in library v0.3)

    /** Make the robot move forward for a predetermined amount of time
    * @param speed - Sets the motors to the specified speed (range -1.0 for max. forward to 1.0 for max. reverse)
    * @param microseconds - The duration to keep moving
    * @param brake - If set to 1, the brake instruction will be applied at the end of the move, else motors are just set to stop
    */
    void time_based_forward(float speed, int microseconds, char brake);

    /** Make the robot turn for a predetermined amount of time
    * @param speed - Sets the turning speed (range -1.0 for max. counter-clockwise to 1.0 for max. clockwise)
    * @param microseconds - The duration to keep moving
    * @param brake - If set to 1, the brake instruction will be applied at the end of the move, else motors are just set to stop
    */
    void time_based_turn(float speed, int microseconds, char brake);

    int time_based_turn_degrees(float speed, float degrees, char brake);
    float get_maximum_turn_angle(int microseconds);
    int get_time_based_turn_time(float speed, float degrees);

private:
    void IF_check_time_for_existing_time_based_action();
    void IF_end_time_based_action();
    void IF_update_motors();
    float IF_calibrated_left_speed(float speed);
    float IF_calibrated_right_speed(float speed);
    //float IF_calibrated_speed(float speed);

};
#endif