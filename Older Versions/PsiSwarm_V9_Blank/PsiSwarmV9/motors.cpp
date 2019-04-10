/* University of York Robotics Laboratory PsiSwarm Library: Motor Functions Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: motors.cpp
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

Timeout time_based_action_timeout;
char brake_when_done = 0;

void Motors::set_left_motor_speed(float speed)
{
    motor_left_speed = speed;
    motor_left_brake = 0;
    IF_update_motors();
}

void Motors::set_right_motor_speed(float speed)
{
    motor_right_speed = speed;
    motor_right_brake = 0;
    IF_update_motors();
}

void Motors::brake_left_motor()
{
    motor_left_speed = 0;
    motor_left_brake = 1;
    IF_update_motors();
}

void Motors::brake_right_motor()
{
    motor_right_speed = 0;
    motor_right_brake = 1;
    IF_update_motors();
}

void Motors::brake()
{
    motor_left_speed = 0;
    motor_right_speed = 0;
    motor_left_brake = 1;
    motor_right_brake = 1;
    IF_update_motors();
}

void Motors::stop()
{
    motor_left_speed = 0;
    motor_right_speed = 0;
    motor_left_brake = 0;
    motor_right_brake = 0;
    IF_update_motors();
}

void Motors::forward(float speed)
{
    motor_left_speed = speed;
    motor_right_speed = speed;
    motor_left_brake = 0;
    motor_right_brake = 0;
    IF_update_motors();
}

void Motors::backward(float speed)
{
    motor_left_speed = -speed;
    motor_right_speed = -speed;
    motor_left_brake = 0;
    motor_right_brake = 0;
    IF_update_motors();
}

void Motors::turn(float speed)
{
    //A positive turn is clockwise
    motor_left_speed = speed;
    motor_right_speed = -speed;
    motor_left_brake = 0;
    motor_right_brake = 0;
    IF_update_motors();
}

//Forward for a set period of time
void Motors::time_based_forward(float speed, int microseconds, char brake)
{
    //Check if a current time based action is running - if it is, throw a warning and cancel its timeout
    IF_check_time_for_existing_time_based_action();

    //Start moving
    forward(speed);
    brake_when_done = brake;
    time_based_action_timeout.attach_us(this,&Motors::IF_end_time_based_action,microseconds);
}

//Turn for a set period of time
void Motors::time_based_turn(float speed, int microseconds, char brake)
{
    //Check if a current time based action is running - if it is, throw a warning and cancel its timeout
    IF_check_time_for_existing_time_based_action();

    //Start turning
    turn(speed);
    brake_when_done = brake;
    time_based_action_timeout.attach_us(this,&Motors::IF_end_time_based_action,microseconds);
}

//Returns the limit of degrees available to turn in given time
float Motors::get_maximum_turn_angle(int microseconds)
{
    //We can turn at about 270 degrees per second at full speed
    return (microseconds * 0.00027);
}

//Return the time in microseconds that performing the turn will take
int Motors::get_time_based_turn_time(float speed, float degrees)
{
    //Check sign of degrees
    if(degrees < 0) degrees =- degrees;

    //Main calculation for turn time
    float turn_time = degrees / ((290 * speed));

    //Add a hard offset of 4ms to account for start\stop time
    if(degrees > 4) {
        turn_time += 0.004;
    } else turn_time += 0.002;

    // Add offset for slow speed
    if(speed<0.31) {
        float mul_fact = 0.31 - speed;
        if(mul_fact < 0) mul_fact = 0;
        mul_fact /= 2;
        mul_fact += 1;
        turn_time *= mul_fact;
    }

    // Add offset for short turns
    if(degrees < 360) {
        float short_offset_multiplier = 1.0 + (0.9 / degrees);
        turn_time *= short_offset_multiplier;
    }

    // Convert to uS
    turn_time *= 1000000;

    return (int) turn_time;
}

//Turn the robot a set number of degrees [using time estimation to end turn]
int Motors::time_based_turn_degrees(float speed, float degrees, char brake)
{
    if(speed < 0 || speed > 1 || degrees == 0) {
        psi.debug("Invalid values to time based turn: speed=%f degrees=$f\n",speed,degrees);
        return 0;
    } else {
        //Check if a current time based action is running - if it is, throw a warning and cancel its timeout
        IF_check_time_for_existing_time_based_action();

        //Calculate turn time using get_time_based_turn_time
        int turn_time = get_time_based_turn_time(speed,degrees);

        //Set correct turn direction (-degrees is a counter-clockwise turn)
        if(degrees < 0) {
            degrees=-degrees;
            speed=-speed;
        }

        //Start turning
        turn(speed);

        brake_when_done = brake;
        time_based_action_timeout.attach_us(this,&Motors::IF_end_time_based_action,turn_time);
        return turn_time;
    }
}

//Check if a current time based action is running - if it is, throw a warning and cancel its timeout
void Motors::IF_check_time_for_existing_time_based_action()
{
    if(time_based_motor_action == 1) {
        time_based_action_timeout.detach();
        psi.debug("WARNING: New time-based action called before previous action finished!\n");
    } else time_based_motor_action = 1;
}

void Motors::IF_end_time_based_action()
{
    if(brake_when_done == 1)brake();
    else stop();
    time_based_motor_action = 0;
}

void Motors::IF_update_motors()
{
    if(motor_left_speed > 1.0) {
        motor_left_speed = 1.0;
        //Throw exception...
    }
    if(motor_right_speed > 1.0) {
        motor_right_speed = 1.0;
        //Throw exception...
    }
    if(motor_left_speed < -1.0) {
        motor_left_speed = -1.0;
        //Throw exception...
    }
    if(motor_right_speed < -1.0) {
        motor_right_speed = -1.0;
        //Throw exception...
    }
    if(motor_left_brake) {
        motor_left_f.write(1);
        motor_left_r.write(1);
        if(motor_left_speed!=0) {
            motor_left_speed = 0;
            //Throw exception...
        }
    } else {
        if(motor_left_speed >= 0) {
            motor_left_f.write(0);
            motor_left_r.write(IF_calibrated_left_speed(motor_left_speed));

        } else {
            motor_left_r.write(0);
            motor_left_f.write(IF_calibrated_left_speed(-motor_left_speed));
        }
    }
    if(motor_right_brake) {
        motor_right_f.write(1);
        motor_right_r.write(1);
        if(motor_right_speed!=0) {
            motor_right_speed = 0;
            //Throw exception...
        }
    } else {
        if(motor_right_speed >= 0) {
            motor_right_f.write(0);
            motor_right_r.write(IF_calibrated_right_speed(motor_right_speed));
        } else {
            motor_right_r.write(0);
            motor_right_f.write(IF_calibrated_right_speed(-motor_right_speed));
        }
    }

}


float Motors::IF_calibrated_left_speed(float speed)
{
    if(speed==0) return 0;
    //Takes the input value, adds an offset if OFFSET_MOTORS enabled and weights the value if USE_MOTOR_CALIBRATION enabled
    float adjusted  = speed;
    if(use_motor_calibration == 1) {
        adjusted *= left_motor_calibration_value;
    }
    if(OFFSET_MOTORS == 1 || use_motor_calibration == 1){
        adjusted*=(1.0-left_motor_stall_offset);
        adjusted+=left_motor_stall_offset;   
    }
    if(adjusted<0) return 0;
    if(adjusted>1) return 1;
    return adjusted;
}

float Motors::IF_calibrated_right_speed(float speed)
{
    if(speed==0) return 0;
    //Takes the input value, adds an offset if OFFSET_MOTORS enabled and weights the value if USE_MOTOR_CALIBRATION enabled
    float adjusted = speed;
    if(use_motor_calibration == 1) {
        adjusted *= right_motor_calibration_value;
    }
    if(OFFSET_MOTORS == 1 || use_motor_calibration == 1){
        adjusted*=(1.0-right_motor_stall_offset);
        adjusted+=right_motor_stall_offset;   
    }
    
    if(adjusted<0) return 0;
    if(adjusted>1) return 1;
    return adjusted;
}

/* Remove from version 0.9 as each motor has unique offset
float Motors::IF_calibrated_speed(float speed)
{
    if(speed == 0) return 0;
    //Converts an input value to take account of the stall speed of the motor; aims to produce a more linear speed
    float adjusted = speed;
    if(OFFSET_MOTORS) {
        adjusted*=0.8f;
        adjusted+=0.2;
    }
    return adjusted;
}
*/


void Motors::calibrate_motors (void)
{
    char restore_use_motor_calibration = use_motor_calibration;
    //Turn off motor calibration whilst testing values!
    use_motor_calibration = 0;
    

    pc.printf("\nMotor Calibration Routine Started\n");
    display.clear_display();
    display.write_string("Starting motor");
    display.set_position(1,0);
    display.write_string("calibration...");
    wait(1);
    char left_offset = 0;
    char right_offset = 0;
    float left_calibration_value = 1;
    float right_calibration_value = 1;
    for(int i=0;i<2;i++){
        display.clear_display();
        if(i==0)display.write_string("LEFT MOTOR");
        else display.write_string("RIGHT MOTOR");
        display.set_position(1,0);
        display.write_string("STALL SPEED");
        wait(1);
        wait(1);
        while(i2c_setup.IF_get_switch_state() != 0) {
          display.clear_display();
          display.set_position(1,0);
          display.write_string("RELEASE SWITCH!");
          wait(0.1);
        }
        display.set_position(1,0);
        display.write_string("                ");
        char a_msg [17];
        char cutoff = 0;
        for(int j=5;j<40;j++){
            display.set_position(1,0);
            sprintf(a_msg,"%d ",j);
            display.write_string(a_msg);
            float target_speed = j * 0.01;
            if(i==0) set_left_motor_speed(target_speed);
            else set_right_motor_speed(target_speed);
            char s_count = 0;
            char switch_state = i2c_setup.IF_get_switch_state();
            while(switch_state == 0 && s_count < 10) {
              s_count ++;
              switch_state = i2c_setup.IF_get_switch_state();
              wait(0.1);
            }
            if(s_count < 10){
                cutoff = j; 
                break;
            }
            motors.brake();
            wait(0.05);
        }
        if (cutoff > 0){
            if(i==0) left_offset = cutoff;
            else right_offset = cutoff;
            pc.printf("Motor stall speed indicated as %f\n",(cutoff * 0.01f));   
        }else pc.printf("Error: Button not pressed\n");
        
    }


    display.clear_display();
    display.set_position(0,0);
    display.write_string("^ REJECT");
    display.set_position(1,2);
    display.write_string("ACCEPT");
    char switch_pos = i2c_setup.IF_get_switch_state();
    while(switch_pos != 1 && switch_pos != 2) switch_pos = i2c_setup.IF_get_switch_state();
    if(switch_pos == 2) {
        pc.printf("\nChanges accepted:  Updating firmware values in EEPROM\n");
        display.clear_display();
        display.set_position(0,0);
        display.write_string("UPDATING");
        display.set_position(1,0);
        display.write_string("FIRMWARE");
        wait(0.5);
        
        eprom.IF_write_motor_calibration_values(left_calibration_value, left_offset, right_calibration_value, right_offset);
        wait(1.5);
    } else {
        pc.printf("\nChanges rejected.\n");
    }
    display.clear_display();
    wait(0.5);
    pc.printf("Motor calibration routine complete\n\n");
    //Restore motor calibration parameters
    use_motor_calibration = restore_use_motor_calibration;
}


void Motors::init_motors()
{
    // Motor PWM outputs work optimally at 25kHz frequency
    motor_left_f.period_us(40);
    motor_right_f.period_us(40);
    motor_left_r.period_us(40);
    motor_right_r.period_us(40);
    motor_left_speed = 0;
    motor_right_speed = 0;
    motor_left_brake = 0;
    motor_right_brake = 0;
    IF_update_motors();
}