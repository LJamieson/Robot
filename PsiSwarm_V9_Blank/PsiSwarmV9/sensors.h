/* University of York Robotics Laboratory PsiSwarm Library: Sensor Functions Header File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: sensors.h
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.8
 *
 * June 2017
 *
 */

#ifndef SENSORS_H
#define SENSORS_H

/**
 * Sensors class
 * Functions to read values from the Psi Swarm infrared, ultrasonic, temperature and power sensors
 *
 * Example:
 * @code
 * #include "psiswarm.h"
 *
 * int main() {
 *     init();
 *
 * }
 * @endcode
*/
class Sensors
{
public:

    /** Returns the current battery voltage for the robot
     * @return The voltage (in V); this should range between 3.5V for a discharged battery and 4.2V for a fully charged battery
     */
    float get_battery_voltage ( void );

    /** Returns the current being used by the robot
     * @return The current (in A)
     */
    float get_current ( void );

    /** Returns the voltage sensed from the DC input (post rectification)
     * @return The voltage (in V); note some back-voltage from the battery is expected even when no DC input is detected
     */
    float get_dc_voltage ( void );

    /** Returns the temperature sensed by the digital thermometer placed near the front of the MBED socket
     * @return The temperature (in degrees C)
     */
    float get_temperature ( void );

    /** Enables a 10Hz ticker that automatically takes readings from the SRF-02 ultrasonic sensor (if fitted)
     */
    void enable_ultrasonic_ticker( void );

    /** Disables the ultrasonic ticker
     */
    void disable_ultrasonic_ticker( void );

    /** Sends a message to SRF-02 ultrasonic sensor (if fitted) to instruct it to take a new reading.  The result is available approx 70ms later
     */
    void update_ultrasonic_measure ( void );
    void IF_read_ultrasonic_measure ( void );
    
    /** Stores the raw ADC values for all 8 IR side-facing sensors without enabling IR emitters
    */
    void store_background_raw_ir_values ( void );
    
    /** Stores the raw ADC values for all 8 IR side-facing sensors after enabling IR emitters
    */
    void store_illuminated_raw_ir_values ( void );
    
    /** Stores the raw ADC values for all 8 IR side-facing sensors both before and after enabling IR emitters
    * Calls store_background_raw_ir_values() then store_illuminated_raw_ir_values()
    */   
    void store_ir_values ( void );
    
    /** Returns the stored value of the non-illuminated side-facing IR sensor value based on last call of store_background_raw_ir_values
    * Call either store_ir_values() or store_background_raw_ir_values() before using this function to update reading
    * @param index - The index of the sensor to read (range 0 to 7, clockwise around robot from front-right)
    * @return Unsigned short of background IR reading (range 0 to 4095)
    */
    unsigned short get_background_raw_ir_value ( char index );
    
    /** Returns the stored value of the illuminated side-facing IR sensor value based on last call of store_illuminated_raw_ir_values
    * Call either store_ir_values() or store_illuminated_raw_ir_values() before using this function to update reading
    * @param index - The index of the sensor to read (range 0 to 7, clockwise around robot from front-right)
    * @return Unsigned short of illuminated IR reading (range 0 to 4095)
    */
    unsigned short get_illuminated_raw_ir_value ( char index );

    /** Returns the subtraction of the background side IR value from the reflection based on last call of store_ir_values()
    * For most purposes this is the best method of detected obstacles etc as it mitigates for varying background levels of IR
    * @param index - The index of the sensor to read (range 0 to 7, clockwise around robot from front-right)
    * @return Unsigned short of compensated ir value (illuminated value - background value) (range 0 to 4095)
    */
    unsigned short calculate_side_ir_value ( char index );
    
    /** Enables the IR emitter then returns the value of the given side-facing IR sensor
    * This function is used when just one sensor is needed to be read; in general using store_ir_values() and get_illuminated_raw_ir_value(index) is preferable
    * @param index - The index of the sensor to read (range 0 to 7, clockwise around robot from front-right)
    * @return Unsigned short of illuminated IR reading (range 0 to 4095)
    */
    unsigned short read_illuminated_raw_ir_value ( char index ) ;

    /** Stores the raw ADC values for all 5 base IR sensors without enabling IR emitters
    */
    void store_background_base_ir_values ( void );
    
    /** Stores the raw ADC values for all 5 base IR sensors after enabling IR emitters
    */
    void store_illuminated_base_ir_values ( void );

    /** Stores the raw ADC values for all 5 base IR sensors both before and after enabling IR emitters
    * Calls store_background_base_ir_values() then store_illuminated_base_ir_values()
    */   
    void store_base_ir_values ( void );
    
    /** Returns the stored value of the non-illuminated base IR sensor value based on last call of store_background_base_ir_values
    * Call either store_base_ir_values() or store_background_base_ir_values() before using this function to update reading
    * @param index - The index of the sensor to read (range 0 to 4, sensor from left to right viewed from above - 2 is in middle of front)
    * @return Unsigned short of background IR reading (range 0 to 4095)
    */
    unsigned short get_background_base_ir_value ( char index );
    
    /** Returns the stored value of the illuminated base IR sensor value based on last call of store_illuminated_base_ir_values
    * Call either store_base_ir_values() or store_illuminated_base_ir_values() before using this function to update reading
    * @param index - The index of the sensor to read (range 0 to 4, sensor from left to right viewed from above - 2 is in middle of front)
    * @return Unsigned short of illuminated IR reading (range 0 to 4095)
    */
    unsigned short get_illuminated_base_ir_value ( char index );
    
    
    /** Returns the subtraction of the background base IR value from the reflection based on last call of store_base_ir_values()
    * For most purposes this is the best method of getting uncalibrated values from the base IR sensor as it mitigates for background levels of IR
    * @param index - The index of the sensor to read (range 0 to 4, sensor from left to right viewed from above - 2 is in middle of front)
    * @return Unsigned short of compensated ir value (illuminated value - background value) (range 0 to 4095)
    */
    unsigned short calculate_base_ir_value ( char index );
    
    /** Returns the calibrated base IR value adjusted for light\dark calibration values 
    * Range is 0.0 to 1.0 where 0.1 is calibrated black value and 0.9 calibrated white value for given sensor
    * If sensor calibration has not been performed and stored in EEPROM used pretested values stored in settings.h
    * NB IR sensor is very material dependant (eg matt paper vs glossy paper): values tested using basic 80gsm copier paper
    * @param index - The index of the sensor to read (range 0 to 4, sensor from left to right viewed from above - 2 is in middle of front)
    * @return Float of compensated ir value (range 0-1, black value = 0.1, white value = 0.9)
    */
    float get_calibrated_base_ir_value ( char index );
    
    /** Determines the position of a black line on a white surface using the base IR sensors
    * Uses the calibrated sensor values 
    *
    * Sets global variable 'line_found' to either 1 if line detected or 0 if line not found
    * Sets global variable 'line_position' (float) to a value ranging from -1.0 (line fully left) to 1.0 (line fully right)
    * [Should range from about -0.5 to 0.5 when center sensor can see line]
    */ 
    void store_line_position (float line_threshold);
    void store_line_position ( void );
    
    
    // The following functions are used for special programs or to retain backwards compatability but are not documented as part of the API
    
    void store_reflected_ir_distances ( void );
    float read_reflected_ir_distance ( char index );
    float get_reflected_ir_distance ( char index );
    float calculate_reflected_distance ( unsigned short background_value, unsigned short illuminated_value );
    int get_bearing_from_ir_array ( unsigned short * ir_sensor_readings);

    void store_line_position_old ( void );
    void calibrate_base_sensors ( void );
    void IF_set_base_calibration_values(int bir1w, int bir2w, int bir3w, int bir4w, int bir5w, int bir1b, int bir2b, int bir3b, int bir4b, int bir5b);
    const char * IF_get_base_calibration_values_string( void );

};
#endif