/* University of York Robotics Laboratory PsiSwarm Library: Sensor Functions Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: sensors.cpp
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



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ultrasonic Sensor (SRF02) Functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// The ultrasonic sensor needs a start command to be sent: this is done by calling update_ultrasonic_measure().
// It can be set to automatically refresh at 10Hz by called enable_ultrasonic_ticker and disable with disabled_ultrasonic_ticker

void Sensors::enable_ultrasonic_ticker()
{
    ultrasonic_ticker.attach_us(this,&Sensors::update_ultrasonic_measure,100000);
}

void Sensors::disable_ultrasonic_ticker()
{
    ultrasonic_ticker.detach();
}

void Sensors::update_ultrasonic_measure()
{
    if(waiting_for_ultrasonic == 0) {
        waiting_for_ultrasonic = 1;
        if(has_ultrasonic_sensor) {
            char command[2];
            command[0] = 0x00;                              // Set the command register
            command[1] = 0x51;                          // Get result is centimeters
            primary_i2c.write(ULTRASONIC_ADDRESS, command, 2);              // Send the command to start a ranging burst
        }
        ultrasonic_timeout.attach_us(this,&Sensors::IF_read_ultrasonic_measure,70000);
    } else {
        psi.debug("WARNING:  Ultrasonic sensor called too frequently.\n");
    }
}

void Sensors::IF_read_ultrasonic_measure()
{
    if(has_ultrasonic_sensor) {
        char command[1];
        char result[2];
        command[0] = 0x02;                           // The start address of measure result
        primary_i2c.write(ULTRASONIC_ADDRESS, command, 1, 1);           // Send address to read a measure
        primary_i2c.read(ULTRASONIC_ADDRESS, result, 2);                // Read two byte of measure
        ultrasonic_distance = (result[0]<<8)+result[1];
    } else ultrasonic_distance = 0;
    ultrasonic_distance_updated = 1;
    waiting_for_ultrasonic = 0;
    //psi.debug("US:%d cm\n",ultrasonic_distance);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Additional Sensing Functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Sensors::get_temperature()
{
    if(has_temperature_sensor)return i2c_setup.IF_read_from_temperature_sensor();
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Voltage Sensing Functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float Sensors::get_battery_voltage ()
{
    float raw_value = vin_battery.read();
    return raw_value * 4.95;
}

float Sensors::get_current ()
{
    // Device uses a INA211 current sense monitor measuring voltage drop across a 2mOhm resistor
    // Device gain = 500
    float raw_value = vin_current.read();
    return raw_value * 3.3;
}

float Sensors::get_dc_voltage ()
{
    float raw_value = vin_dc.read();
    return raw_value * 6.6;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// IR Sensor Functions
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int cv_bir_w[5],cv_bir_b[5];

// Estimates the distance to an obstacle from one of the IR sensors, defined by index (range 0-7).
// The value is converted to an approximate distance in millimetres, or 100.0 if no obstacle found.
// NB This function is preserved for code-compatability and cases where only a single reading is needed
// In many cases it is preferable to call store_reflected_ir_distances() to save all 8 values then read using get_reflected_ir_distance()
float Sensors::read_reflected_ir_distance ( char index )
{
    // Sanity check
    if(index>7) return 0.0;

    //1.  Read the ADC value without IR emitter on; store in the background_ir_values[] array
    background_ir_values [index] = i2c_setup.IF_read_IR_adc_value(1, index );

    //2.  Enable the relevant IR emitter by turning on its pulse output
    switch(index) {
        case 0:
        case 1:
        case 6:
        case 7:
            i2c_setup.IF_set_IR_emitter_output(0, 1);
            break;
        case 2:
        case 3:
        case 4:
        case 5:
            i2c_setup.IF_set_IR_emitter_output(1, 1);
            break;
    }
    wait_us(ir_pulse_delay);

    //3.  Read the ADC value now IR emitter is on; store in the illuminated_ir_values[] array
    illuminated_ir_values [index] = i2c_setup.IF_read_IR_adc_value (1, index );

    //4.  Turn off IR emitter
    switch(index) {
        case 0:
        case 1:
        case 6:
        case 7:
            i2c_setup.IF_set_IR_emitter_output(0, 0);
            break;
        case 2:
        case 3:
        case 4:
        case 5:
            i2c_setup.IF_set_IR_emitter_output(1, 0);
            break;
    }

    //5.  Estimate distance based on 2 values using calculate_reflected_distances(); store in reflected_ir_distances()
    reflected_ir_distances [index] = calculate_reflected_distance( background_ir_values [index], illuminated_ir_values [index]);
    ir_values_stored = 1;
    return reflected_ir_distances [index];
}


// Returns the stored value of the reflected obstacle based on last call of either read_reflected_ir_distance or store_reflected_distances
float Sensors::get_reflected_ir_distance ( char index )
{
    // Sanity check: check range of index and that values have been stored
    if (index>7 || ir_values_stored == 0) return 0.0;
    return reflected_ir_distances[index];
}

// Returns the stored value of the non-illuminated sensor based on last call of store_background_raw_ir_values
unsigned short Sensors::get_background_raw_ir_value ( char index )
{
    // Sanity check: check range of index and that values have been stored
    if (index>7 || ir_values_stored == 0) return 0.0;
    return background_ir_values[index];
}

// Returns the stored value of the illuminated sensor based on last call of store_illuminated_raw_ir_values
unsigned short Sensors::get_illuminated_raw_ir_value ( char index )
{
    // Sanity check: check range of index and that values have been stored
    if (index>7 || ir_values_stored == 0) return 0.0;
    return illuminated_ir_values[index];
}

// Stores the reflected distances for all 8 IR sensors
void Sensors::store_reflected_ir_distances ( void )
{
    store_ir_values();
    for(int i=0; i<8; i++) {
        reflected_ir_distances [i] = calculate_reflected_distance( background_ir_values [i], illuminated_ir_values [i]);
    }
}

// Stores the background and illuminated values for all 8 IR sensors
void Sensors::store_ir_values ( void )
{
    store_background_raw_ir_values();
    store_illuminated_raw_ir_values();
}

// Stores the raw ADC values for all 8 IR sensors without enabling IR emitters
void Sensors::store_background_raw_ir_values ( void )
{
    ir_values_stored = 1;
    for(int i=0; i<8; i++) {
        background_ir_values [i] = i2c_setup.IF_read_IR_adc_value(1,i);
    }
}

// Stores the raw ADC values for all 8 IR sensors with a 500us emitter pulse
void Sensors::store_illuminated_raw_ir_values ( void )
{
    //1.  Enable the front IR emitters and store the values
    i2c_setup.IF_set_IR_emitter_output(0, 1);
    wait_us(ir_pulse_delay);
    illuminated_ir_values [0] = i2c_setup.IF_read_IR_adc_value(1,0);
    illuminated_ir_values [1] = i2c_setup.IF_read_IR_adc_value(1,1);
    illuminated_ir_values [6] = i2c_setup.IF_read_IR_adc_value(1,6);
    illuminated_ir_values [7] = i2c_setup.IF_read_IR_adc_value(1,7);
    i2c_setup.IF_set_IR_emitter_output(0, 0);

    //2.  Enable the rear+side IR emitters and store the values
    i2c_setup.IF_set_IR_emitter_output(1, 1);
    wait_us(ir_pulse_delay);
    illuminated_ir_values [2] = i2c_setup.IF_read_IR_adc_value(1,2);
    illuminated_ir_values [3] = i2c_setup.IF_read_IR_adc_value(1,3);
    illuminated_ir_values [4] = i2c_setup.IF_read_IR_adc_value(1,4);
    illuminated_ir_values [5] = i2c_setup.IF_read_IR_adc_value(1,5);
    i2c_setup.IF_set_IR_emitter_output(1, 0);
}

// Converts a background and illuminated value into a distance
float Sensors::calculate_reflected_distance ( unsigned short background_value, unsigned short illuminated_value )
{
    float approximate_distance = 4000 + background_value - illuminated_value;
    if(approximate_distance < 0) approximate_distance = 0;

    // Very approximate: root value, divide by 2, approx distance in mm
    approximate_distance = sqrt (approximate_distance) / 2.0;

    // Then add adjustment value if value >27
    if(approximate_distance > 27) {
        float shift = pow(approximate_distance - 27,3);
        approximate_distance += shift;
    }
    if(approximate_distance > 90) approximate_distance = 100.0;
    return approximate_distance;
}

// Returns the illuminated raw sensor value for the IR sensor defined by index (range 0-7); turns on the emitters for a 500us pulse
unsigned short Sensors::read_illuminated_raw_ir_value ( char index )
{
    //This function reads the IR strength when illuminated - used for PC system debugging purposes
    //1.  Enable the relevant IR emitter by turning on its pulse output
    switch(index) {
        case 0:
        case 1:
        case 6:
        case 7:
            i2c_setup.IF_set_IR_emitter_output(0, 1);
            break;
        case 2:
        case 3:
        case 4:
        case 5:
            i2c_setup.IF_set_IR_emitter_output(1, 1);
            break;
    }
    wait_us(ir_pulse_delay);
    //2.  Read the ADC value now IR emitter is on
    unsigned short strong_value = i2c_setup.IF_read_IR_adc_value( 1,index );
    //3.  Turn off IR emitter
    switch(index) {
        case 0:
        case 1:
        case 6:
        case 7:
            i2c_setup.IF_set_IR_emitter_output(0, 0);
            break;
        case 2:
        case 3:
        case 4:
        case 5:
            i2c_setup.IF_set_IR_emitter_output(1, 0);
            break;
    }
    return strong_value;
}

// Base IR sensor functions


// Returns the stored value of the non-illuminated sensor based on last call of store_background_base_ir_values
unsigned short Sensors::get_background_base_ir_value ( char index )
{
    // Sanity check: check range of index and that values have been stored
    if (index>4 || base_ir_values_stored == 0) return 0.0;
    return background_base_ir_values[index];
}

// Returns the stored value of the illuminated sensor based on last call of store_illuminated_base_ir_values
unsigned short Sensors::get_illuminated_base_ir_value ( char index )
{
    // Sanity check: check range of index and that values have been stored
    if (index>4 || base_ir_values_stored == 0) return 0.0;
    return illuminated_base_ir_values[index];
}

// Stores the reflected distances for all 5 base IR sensors
void Sensors::store_base_ir_values ( void )
{
    store_background_base_ir_values();
    store_illuminated_base_ir_values();
    //for(int i=0;i<5;i++){
    //  reflected_ir_distances [i] = calculate_reflected_distance( background_base_ir_values [i], illuminated_base_ir_values [i]);
    //}
}

// Stores the raw ADC values for all 5 base IR sensors without enabling IR emitters
void Sensors::store_background_base_ir_values ( void )
{
    base_ir_values_stored = 1;
    for(int i=0; i<5; i++) {
        background_base_ir_values [i] = i2c_setup.IF_read_IR_adc_value(2,i);
    }
}

// Stores the raw ADC values for all 5 base IR sensors with a 500us emitter pulse
void Sensors::store_illuminated_base_ir_values ( void )
{
    //1.  Enable the base IR emitters and store the values
    i2c_setup.IF_set_IR_emitter_output(2, 1);
    wait_us(base_ir_pulse_delay);
    for(int i=0; i<5; i++) {
        illuminated_base_ir_values [i] = i2c_setup.IF_read_IR_adc_value(2,i);
    }

    i2c_setup.IF_set_IR_emitter_output(2, 0);
}

// Routine to store detected line position in a similar format to the used on 3Pi\m3Pi\PiSwarm
// Old version (doesn't use calibration values)
void Sensors::store_line_position_old ( )
{
    // Store background and reflected base IR values
    store_base_ir_values();
    int h_value[5];
    int line_threshold = 1000;
    int line_threshold_hi = 2000;
    char count = 0;
    line_found = 0;
    line_position = 0;
    for(int i=0; i<5; i++) {
        if(get_background_base_ir_value(i) > get_illuminated_base_ir_value(i)) h_value[i]=0;
        else h_value[i] = get_illuminated_base_ir_value(i) - get_background_base_ir_value(i);
        if(h_value[i] < line_threshold) count++;
    }
    if(count == 1) {
        line_found = 1;
        if(h_value[0] < line_threshold) {
            line_position = -1;
            if(h_value[1] < line_threshold_hi) line_position = -0.8;
        }

        if (h_value[1] < line_threshold) {
            line_position = -0.5 + (0.00005 * h_value[0]) - (0.0001 * h_value[2]);;
        }
        if(h_value[2] < line_threshold) {
            line_position = (0.00005 * h_value[1]) - (0.0001 * h_value[3]);
        }
        if(h_value[3] < line_threshold) {
            line_position = 0.5 + (0.00005 * h_value[2]) - (0.0001 * h_value[4]);;
        }
        if(h_value[4] < line_threshold) {
            line_position = 1;
            if(h_value[3] < line_threshold_hi) line_position = 0.8;
        }
    }
    if(count == 2) {
        if(h_value[0] && h_value[1] < line_threshold) {
            line_found = 1;
            line_position = -0.6;
        }

        if(h_value[1] && h_value[2] < line_threshold) {
            line_found = 1;
            line_position = -0.4;
        }

        if(h_value[2] && h_value[3] < line_threshold) {
            line_found = 1;
            line_position = 0.4;
        }

        if(h_value[3] && h_value[4] < line_threshold) {
            line_found = 1;
            line_position = 0.6;
        }
    }
}

// New version (uses calibration values)
void Sensors::store_line_position (void)
{
    store_line_position(0.5);
}

// Routine to store detected line position in a similar format to the used on 3Pi\m3Pi\PiSwarm
void Sensors::store_line_position (float line_threshold)
{
    // Store background and reflected base IR values
    store_base_ir_values();
    float calibrated_values[5];
    float adjust_values[5];
    char count = 0;
    for(int i=0; i<5; i++) {
        calibrated_values[i] = get_calibrated_base_ir_value(i);
        if(calibrated_values[i] < line_threshold) count ++;
        adjust_values[i] = 1 - calibrated_values[i];
        adjust_values[i] *= adjust_values[i];
        adjust_values[i] *= 0.5f;
        
    }

    line_found = 0;
    line_position = 0;

    if(count == 1) {
        line_found = 1;
        if(calibrated_values[0] < line_threshold) {
            line_position = -1 + adjust_values[1];
        }

        if (calibrated_values[1] < line_threshold) {
            line_position = -0.5 - adjust_values[0] + adjust_values[2];
        }
        if(calibrated_values[2] < line_threshold) {
            line_position = 0 - adjust_values[1] + adjust_values[3];
        }
        if(calibrated_values[3] < line_threshold) {
            line_position = 0.5 - adjust_values[2] + adjust_values[4];
        }
        if(calibrated_values[4] < line_threshold) {
            line_position = 1 - adjust_values[3];
        }
    }

    if(count == 2) {
        if(calibrated_values[0] < line_threshold && calibrated_values[1] < line_threshold) {
            line_found = 1;
            line_position = -0.75 - adjust_values[0] + adjust_values[1];
        }

        if(calibrated_values[1] < line_threshold && calibrated_values[2] < line_threshold) {
            line_found = 1;
            line_position = -0.25 -adjust_values[1] + adjust_values[2];
        }

        if(calibrated_values[2] < line_threshold && calibrated_values[3] < line_threshold) {
            line_found = 1;
            line_position = 0.25 - adjust_values[2] + adjust_values[3];
        }

        if(calibrated_values[3] < line_threshold && calibrated_values[4] < line_threshold) {
            line_found = 1;
            line_position = 0.75 - adjust_values[3] + adjust_values[4];
        }
    }

    if(count == 3) {
        if (calibrated_values[1] < line_threshold && calibrated_values[2] < line_threshold && calibrated_values[3] < line_threshold) {
            line_found = 1;
            line_position = 0 - adjust_values[1] + adjust_values[3];
        }
    }
    if(line_position < -1) line_position = -1;
    if(line_position > 1) line_position = 1;
}

// Returns the subtraction of the background base IR value from the reflection based on last call of store_illuminated_base_ir_values
unsigned short Sensors::calculate_base_ir_value ( char index )
{
    // If the index is not in the correct range or the base IR values have not been stored, return zero
    if (index>4 || base_ir_values_stored == 0) return 0.0;
    // If the reflection value is greater than the background value, return the subtraction
    if(illuminated_base_ir_values[index] > background_base_ir_values[index]) {
        return illuminated_base_ir_values[index] - background_base_ir_values[index];
        //Otherwise return zero
    } else {
        return 0.0;
    }
}

float Sensors::get_calibrated_base_ir_value ( char index )
{
    float uncalibrated_value = (float) calculate_base_ir_value(index);
    float lower = 0.9f * cv_bir_b[index];
    float calibrated_value = uncalibrated_value - lower;
    float upper = 1.1f * (cv_bir_w[index] - lower);
    calibrated_value /= upper;
    if(calibrated_value < 0) calibrated_value = 0;
    if(calibrated_value > 1) calibrated_value = 1;
    return calibrated_value;
}

// Returns the subtraction of the background side IR value from the reflection based on last call of store_illuminated_base_ir_values
unsigned short Sensors::calculate_side_ir_value ( char index )
{
    // If the index is not in the correct range or the base IR values have not been stored, return zero
    if (index>7 || ir_values_stored == 0) return 0.0;
    // If the reflection value is greater than the background value, return the subtraction
    if(illuminated_ir_values[index] > background_ir_values[index]) {
        return illuminated_ir_values[index] - background_ir_values[index];
        //Otherwise return zero
    } else {
        return 0.0;
    }
}

void Sensors::calibrate_base_sensors (void)
{
    char test_colour_sensor = has_base_colour_sensor;
    short white_background[5];
    short white_active[5];
    short black_background[5];
    short black_active[5];
    int white_colour[4];
    int black_colour[4];
    for(int k=0; k<5; k++) {
        white_background[k]=0;
        black_background[k]=0;
        white_active[k]=0;
        black_active[k]=0;
    }

    pc.printf("\nBase Sensor Calibration\n");
    display.clear_display();
    display.write_string("Starting sensor");
    display.set_position(1,0);
    display.write_string("calibration...");
    wait(1);
    display.clear_display();
    display.write_string("Place robot on");
    display.set_position(1,0);
    display.write_string("white surface");
    wait(4);
    display.clear_display();
    display.write_string("Calibrating base");
    display.set_position(1,0);
    display.write_string("IR sensor");
    wait(0.5);
    pc.printf("\n\nWhite Surface IR Results:\n");

    for(int i=0; i<5; i++) {
        wait(0.2);
        store_background_base_ir_values();

        display.set_position(1,9+i);
        display.write_string(".");
        wait(0.2);
        store_illuminated_base_ir_values();
        for(int k=0; k<5; k++) {
            white_background[k]+=  get_background_base_ir_value(k);
            white_active[k] += get_illuminated_base_ir_value(k);
        }
        pc.printf("Sample %d     1: %04d-%04d  2: %04d-%04d  3: %04d-%04d  4: %04d-%04d  5: %04d-%04d\n", (i+1),
                  get_background_base_ir_value(0),            get_illuminated_base_ir_value(0),
                  get_background_base_ir_value(1),            get_illuminated_base_ir_value(1),
                  get_background_base_ir_value(2),            get_illuminated_base_ir_value(2),
                  get_background_base_ir_value(3),            get_illuminated_base_ir_value(3),
                  get_background_base_ir_value(4),            get_illuminated_base_ir_value(4));
    }
    for(int k=0; k<5; k++) {
        white_background[k]/=5;
        white_active[k]/=5;
    }
    pc.printf("Mean results 1: %04d-%04d  2: %04d-%04d  3: %04d-%04d  4: %04d-%04d  5: %04d-%04d\n",
              white_background[0],          white_active[0],
              white_background[1],          white_active[1],
              white_background[2],          white_active[2],
              white_background[3],          white_active[3],
              white_background[4],          white_active[4]);
    wait(0.25);
    char retake_sample = 1;
    if(test_colour_sensor) {
        display.clear_display();
        display.write_string("Calibrating base");
        display.set_position(1,0);
        display.write_string("colour sensor");
        wait(0.2);
        char retake_white_count = 0;

        while(retake_sample == 1 && retake_white_count<3) {
            wait(0.2);
            led.set_base_led(1);
            colour.enable_base_colour_sensor();
            wait(0.03);
            colour.read_base_colour_sensor_values( white_colour );
            colour.disable_base_colour_sensor();
            led.set_base_led(0);
            if(white_colour[1] < 1 || white_colour[1] > 1022 || white_colour[2] < 1 || white_colour[2] > 1022 || white_colour[3] < 1 || white_colour[3] > 1022) {
                pc.printf("Warning: Colour tested exceeded expected range\n");
                retake_white_count ++;
            } else retake_sample=0;
        }
        if(retake_white_count > 2) {
            test_colour_sensor = 0;
            pc.printf("The colour sensor test has produced results outside expected range; check gain settings and test setup");
        } else {
            wait(0.1);
            pc.printf("\n\nWhite Surface Colour Sensor Results:\n");
            pc.printf("BRIGHTNESS:%4d RED:%4d GREEN:%4d BLUE:%d\n",white_colour[0],white_colour[1],white_colour[2],white_colour[3]);
        }
    }
    wait(0.5);

    display.clear_display();
    display.write_string("Place robot on");
    display.set_position(1,0);
    display.write_string("black surface");
    wait(3.5);

    display.clear_display();
    display.write_string("Calibrating base");
    display.set_position(1,0);
    display.write_string("IR sensor");
    wait(0.5);
    pc.printf("\nBlack Surface Results:\n");

    for(int i=0; i<5; i++) {
        wait(0.2);

        store_background_base_ir_values();
        display.set_position(1,9);
        display.write_string(".");
        wait(0.2);
        store_illuminated_base_ir_values();
        for(int k=0; k<5; k++) {
            black_background[k]+=  get_background_base_ir_value(k);
            black_active[k] += get_illuminated_base_ir_value(k);
        }
        pc.printf("Sample %d     1: %04d-%04d  2: %04d-%04d  3: %04d-%04d  4: %04d-%04d  5: %04d-%04d\n", (i+1),
                  get_background_base_ir_value(0),            get_illuminated_base_ir_value(0),
                  get_background_base_ir_value(1),            get_illuminated_base_ir_value(1),
                  get_background_base_ir_value(2),            get_illuminated_base_ir_value(2),
                  get_background_base_ir_value(3),            get_illuminated_base_ir_value(3),
                  get_background_base_ir_value(4),            get_illuminated_base_ir_value(4));
    }
    for(int k=0; k<5; k++) {
        black_background[k]/=5;
        black_active[k]/=5;
    }
    pc.printf("Mean results 1: %04d-%04d  2: %04d-%04d  3: %04d-%04d  4: %04d-%04d  5: %04d-%04d\n",
              black_background[0],          black_active[0],
              black_background[1],          black_active[1],
              black_background[2],          black_active[2],
              black_background[3],          black_active[3],
              black_background[4],          black_active[4]);
    wait(0.25);
    if(test_colour_sensor) {
        display.clear_display();
        display.write_string("Calibrating base");
        display.set_position(1,0);
        display.write_string("colour sensor");
        wait(0.2);
        char retake_black_count = 0;
        retake_sample = 1;
        while(retake_sample == 1 && retake_black_count<3) {
            wait(0.2);
            led.set_base_led(1);
            colour.enable_base_colour_sensor();
            wait(0.03);
            colour.read_base_colour_sensor_values( black_colour );
            colour.disable_base_colour_sensor();
            led.set_base_led(0);
            if(black_colour[1] < 1 || black_colour[1] > 1022 || black_colour[2] < 1 || black_colour[2] > 1022 || black_colour[3] < 1 || black_colour[3] > 1022) {
                pc.printf("Warning: Colour tested exceeded expected range\n");
                retake_black_count ++;
            } else retake_sample=0;
        }
        if(retake_black_count > 2) {
            test_colour_sensor = 0;
            pc.printf("The colour sensor test has produced results outside expected range; check gain settings and test setup");
        } else {
            wait(0.1);
            pc.printf("\n\nBlack Surface Colour Sensor Results:\n");
            pc.printf("BRIGHTNESS:%4d RED:%4d GREEN:%4d BLUE:%d\n",black_colour[0],black_colour[1],black_colour[2],black_colour[3]);
        }
    }
    wait(1);
    while(i2c_setup.IF_get_switch_state() != 0) {
        display.clear_display();
        display.set_position(0,0);
        display.write_string("RELEASE SWITCH!");
        wait(0.1);
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
        pc.printf("- Updating bass IR sensor values\n");
        eprom.IF_write_base_ir_calibration_values(white_active,black_active);
        if(test_colour_sensor == 1) {
            wait(0.5);
            pc.printf("- Updating bass colour sensor values\n");
            eprom.IF_write_base_colour_calibration_values(black_colour,white_colour);
        }
        wait(1.5);
    } else {
        pc.printf("\nChanges rejected.\n");
    }
    display.clear_display();
    wait(0.5);
    pc.printf("Base sensor calibration routine complete\n\n");
}


int Sensors::get_bearing_from_ir_array (unsigned short * ir_sensor_readings)
{
    //out("Getting bearing from array: [%d][%d][%d][%d][%d][%d][%d][%d]\n",ir_sensor_readings[0],ir_sensor_readings[1],ir_sensor_readings[2],ir_sensor_readings[3],ir_sensor_readings[4],ir_sensor_readings[5],ir_sensor_readings[6],ir_sensor_readings[7]);

    float degrees_per_radian = 57.295779513;

    // sin(IR sensor angle) and cos(IR sensor angle) LUT, for all 8 sensors
    float ir_sensor_sin[8] = {0.382683432, 0.923879533, 0.923879533, 0.382683432, -0.382683432, -0.923879533, -0.923879533, -0.382683432};
    float ir_sensor_cos[8] = {0.923879533, 0.382683432, -0.382683432, -0.923879533, -0.923879533, -0.382683432, 0.382683432, 0.923879533};

    float sin_sum = 0;
    float cos_sum = 0;

    for(int i = 0; i < 8; i++) {
        // Use IR sensor reading to weight bearing vector
        sin_sum += ir_sensor_sin[i] * ir_sensor_readings[i];
        cos_sum += ir_sensor_cos[i] * ir_sensor_readings[i];
    }

    float bearing = atan2(sin_sum, cos_sum); // Calculate vector towards IR light source
    bearing *= degrees_per_radian; // Convert to degrees

    //out("Sin sum:%f Cos sum:%f Bearing:%f\n",sin_sum,cos_sum,bearing);

    return (int) bearing;
}

void Sensors::IF_set_base_calibration_values(int bir1w, int bir2w, int bir3w, int bir4w, int bir5w, int bir1b, int bir2b, int bir3b, int bir4b, int bir5b)
{
    cv_bir_w[0] = bir1w;
    cv_bir_w[1] = bir2w;
    cv_bir_w[2] = bir3w;
    cv_bir_w[3] = bir4w;
    cv_bir_w[4] = bir5w;
    cv_bir_b[0] = bir1b;
    cv_bir_b[1] = bir2b;
    cv_bir_b[2] = bir3b;
    cv_bir_b[3] = bir4b;
    cv_bir_b[4] = bir5b;
}
char cv_description[105];
const char * Sensors::IF_get_base_calibration_values_string()
{
    
    sprintf(cv_description,"[BLACK 1:%4d 2:%4d 3:%4d 4:%4d 5:%4d WHITE 1:%4d 2:%4d 3:%4d 4:%4d 5:%4d]",cv_bir_b[0],cv_bir_b[1],cv_bir_b[2],cv_bir_b[3],cv_bir_b[4],cv_bir_w[0],cv_bir_w[1],cv_bir_w[2],cv_bir_w[3],cv_bir_w[4]);
    return cv_description;
}