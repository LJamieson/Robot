/* University of York Robotics Laboratory PsiSwarm Library: Colour Sensors Header File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: colour.h
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


#ifndef COLOUR_H
#define COLOUR_H

#define ENABLE                  0x00
#define ATIME                   0x01
#define CONTROL                 0x0F
#define CDATA                   0x14

/**
 *  The Colour class contains the functions for reading the base-mounted and top-mounted I2C colour sensors (optional).
 *
 * Functions for use with the TAOS TCS34725 and TCS34721 I2C colour sensors
 *
 * Example:
 * @code
 * #include "psiswarm.h"
 *
 * int main() {
 *     init();
 *     colour.start_colour_ticker(150);
       while(1) {
            pc.printf("Colour: %s\n",colour.get_colour_string(colour.get_detected_colour())); 
            wait(0.20);
       }
 * }
 * @endcode
*/
class Colour
{
public:
    /** Set integration time and gain of colour sensor to default values
    *
    */
    void colour_sensor_init();

    /** Set the white\black calibration values for the colour sensor
    *
    */    
    void set_calibration_values(int c_black,int r_black,int g_black,int b_black,int c_white,int r_white,int g_white,int b_white);

    /** Set the gain of the base colour sensor
    *
    * @param gain The gain value for the sensor
    */
    void set_base_colour_sensor_gain(char gain);

    /** Set the integration time constant for the base colour sensor
    *
    * @param gain The gain value for the sensor
    */
    void set_base_colour_sensor_integration_time(char int_time);

    /** Enable the base colour sensor
    */
    void enable_base_colour_sensor(void);

    /** Disable (power-down) the base colour sensor
    */
    void disable_base_colour_sensor(void);

    /** Read the values from the base colour sensor
    *
    * @param Pointer to 3 x int array for r-g-b values
    */
    void read_base_colour_sensor_values(int * store_array);
    
    /** Function enables colour sensor, takes a reading, returns a single int (range -1 to 8)
    *
    * @return Output of identify_colour_from_calibrated_colour_scores - int range -1 to 8
    */
    int detect_colour_once();
    
    /** Returns the most recent identified colour from detect_colour_once or detect_colour_ticker (range -1 to 8)
    *
    * @return Output of identify_colour_from_calibrated_colour_scores - int range -1 to 8
    */  
    int get_detected_colour();

    /** Attempts to identify a colour from a given array of calibrated colour values
    *
    * @param calibrate_colour_array_in : Pointer to the calibrate colour array [output of get_calibrated_colour] 
    * @return int defining detected colour: 0=RED 1=YELLOW 2=GREEN 3=CYAN 4=BLUE 5=MAGENTA 6=WHITE 7=BLACK 8=UNSURE
    */
    int identify_colour_from_calibrated_colour_scores(float * calibrate_colour_array_in);
    
    /** Translate an input array of raw CT,R,G,B values into calibrated, normalised values 
    *
    * @param colour_array_in : 4-element int input array [output of read_base_colour_sensor_values()]
    * @param colour_array_out : Target 4-element float array of normalised values [0=CT, range 0-1  1=R 2=G 3=B, Sum R+G+B=3.0]
    */
    void get_calibrated_colour(int * colour_array_in, float * colour_array_out);
    
    /** Returns a string form of the output of identify_colour_from_calibrated_colour_scores()
    *
    * @param colour_index : Output of identify_colour_from_calibrated_colour_scores()
    * @return 7-character String (eg 0="RED    ")
    */
    const char * get_colour_string(int colour_index); 

    /** Starts a polling ticker that cyclically checks to see if a colour can be detected
    *
    * @param period_ms : The approximate cycle period in milliseconds
    */
    void start_colour_ticker(int period_ms);
    
    /** Stops the polling ticker
    *
    */
    void stop_colour_ticker(void);
    
    

    void IF_colour_ticker_start();
    void IF_colour_ticker_main();
    char IF_check_base_colour_sensor(void);
    int IF_writeSingleRegister( char address, char data );
    int IF_writeMultipleRegisters( char address, char* data, int quantity );
    char IF_readSingleRegister( char address );
    int IF_readMultipleRegisters( char address, char* output, int quantity );
    float IF_roundTowardsZero( const float value );
    const char * IF_get_calibration_values_string(void);
};
#endif