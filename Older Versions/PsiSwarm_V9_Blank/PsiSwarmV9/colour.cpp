/* University of York Robotics Laboratory PsiSwarm Library: Colour Sensors Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: colour.cpp
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
 */


// Base colour sensor is a TCS34725
// Top colour sensor (if fitted) is a TCS34721
#include "psiswarm.h"

int last_detected_colour = -1;

int cs_c_black,cs_r_black,cs_g_black,cs_b_black,cs_c_white,cs_r_white,cs_g_white,cs_b_white;

void Colour::set_calibration_values(int c_black,int r_black,int g_black,int b_black,int c_white,int r_white,int g_white,int b_white){
    cs_c_black = c_black;
    cs_r_black = r_black;
    cs_g_black = g_black;
    cs_b_black = b_black;
    cs_c_white = c_white;
    cs_r_white = r_white;
    cs_g_white = g_white;
    cs_b_white = b_white;
}

void Colour::colour_sensor_init()
{
     colour.set_base_colour_sensor_integration_time(BASE_COLOUR_SENSOR_INTEGRATION_TIME);
     colour.set_base_colour_sensor_gain(BASE_COLOUR_SENSOR_GAIN);   
}

void Colour::read_base_colour_sensor_values(int * store_array)
{
    char buffer[8] = { 0 };
    IF_readMultipleRegisters( CDATA, buffer, 8 );
    store_array[0] = (int)buffer[1] << 8 | (int)buffer[0];
    store_array[1] = (int)buffer[3] << 8 | (int)buffer[2];
    store_array[2] = (int)buffer[5] << 8 | (int)buffer[4];
    store_array[3] = (int)buffer[7] << 8 | (int)buffer[6];
}

void Colour::set_base_colour_sensor_gain(char gain)
{
    char control;
    int ack = 0;
    switch (gain) {
        case 1:
            control = 0;
            break;
        case 4:
            control = 1;
            break;
        case 16:
            control = 2;
            break;
        case 60:
            control = 3;
            break;
        default:
            ack = 2; // 2 used to indicate invalid entry
            break;
    }
    if ( ack != 2 ) {
        ack = IF_writeSingleRegister( CONTROL, control );
    }
}

void Colour::set_base_colour_sensor_integration_time(char int_time)
{
    char atime = 256 - IF_roundTowardsZero( int_time / 2.4 ); // rounding ensures nearest value of atime is used
    int ack = IF_writeSingleRegister( ATIME, atime );
}

float Colour::IF_roundTowardsZero( const float value )
{
    float result = 0;
    if ( ( value >= 0 && ( value - (int)value ) < 0.5 ) || ( value < 0 && ( abs(value) - (int)abs(value) ) >= 0.5 ) ) {
        result = floor(value);
    } else {
        result = ceil(value);
    }
    return result;
}

void Colour::enable_base_colour_sensor(void)
{
    char enable_old = IF_readSingleRegister( ENABLE );
    char enable_new = enable_old | 3; // sets PON (bit 0) and AEN (bit 1) to 1
    int ack = IF_writeSingleRegister( ENABLE, enable_new );
}


void Colour::disable_base_colour_sensor(void)
{
    char enable_old = IF_readSingleRegister( ENABLE );
    char enable_new = enable_old & 252; // sets PON (bit 0) and AEN (bit 1) to 0
    int ack = IF_writeSingleRegister( ENABLE, enable_new );
}

Timeout colour_ticker;
int colour_ticker_period;
int colour_ticker_on = 0;

void Colour::start_colour_ticker(int period_ms)
{
    colour_ticker_on = 1;
    colour_ticker_period = period_ms;
    colour_ticker.attach_us(this,&Colour::IF_colour_ticker_start, 100);
}

void Colour::stop_colour_ticker()
{
    colour_ticker_on = 0;
    colour_ticker.detach();   
}
    
void Colour::IF_colour_ticker_start()
{
    led.set_base_led(1);
    enable_base_colour_sensor();
    colour_ticker.attach_us(this,&Colour::IF_colour_ticker_main, 25000);
}

void Colour::IF_colour_ticker_main()
{
    int rgb_readings [4];
    read_base_colour_sensor_values( rgb_readings );
    disable_base_colour_sensor();
    led.set_base_led(0);
    if(rgb_readings[1] > 0 && rgb_readings[1] < 1024 && rgb_readings[2] > 0 && rgb_readings[2] < 1024 && rgb_readings[3] > 0 && rgb_readings[3] < 1024) {
        float adjusted[4];
        get_calibrated_colour(rgb_readings,adjusted);
        last_detected_colour = identify_colour_from_calibrated_colour_scores(adjusted);
    }
    if(colour_ticker_on == 1)colour_ticker.attach_us(this,&Colour::IF_colour_ticker_start, colour_ticker_period * 1000);
}



int Colour::detect_colour_once()
{
    int rgb_readings [4];
    led.set_base_led(1);
    enable_base_colour_sensor();
    wait(0.03);
    read_base_colour_sensor_values( rgb_readings );
    disable_base_colour_sensor();
    led.set_base_led(0);
    if(rgb_readings[1] < 1 || rgb_readings[1] > 1022 || rgb_readings[2] < 1 || rgb_readings[2] > 1022 || rgb_readings[3] < 1 || rgb_readings[3] > 1022) return -1;
    float adjusted[4];
    get_calibrated_colour(rgb_readings,adjusted);
    last_detected_colour = identify_colour_from_calibrated_colour_scores(adjusted);
    return last_detected_colour;
}

int Colour::get_detected_colour()
{
    return last_detected_colour;
}

const char * Colour::get_colour_string(int colour_index)
{
    switch(colour_index) {
        case 0:
            return "RED    ";
        case 1:
            return "YELLOW ";
        case 2:
            return "GREEN  ";
        case 3:
            return "CYAN   ";
        case 4:
            return "BLUE   ";
        case 5:
            return "MAGENTA";
        case 6:
            return "WHITE  ";
        case 7:
            return "BLACK  ";
    }
    return "-------";
}

void Colour::get_calibrated_colour(int * colour_array_in, float * colour_array_out)
{
    int colour_temp = colour_array_in[0];
    if(colour_temp < cs_c_black) colour_temp = cs_c_black;
    if(colour_temp > cs_c_white) colour_temp = cs_c_white;
    colour_array_out[0] = (colour_temp - cs_c_black) / (float) (cs_c_white - cs_c_black);
    float black_level = 1.0 - colour_array_out[0];
    colour_array_out[1] = ((colour_array_in[1] / (float)cs_r_white) * colour_array_out[0]) + ((colour_array_in[1] / (float)cs_r_black) * black_level);
    colour_array_out[2] = ((colour_array_in[2] / (float)cs_g_white) * colour_array_out[0]) + ((colour_array_in[2] / (float)cs_g_black) * black_level);
    colour_array_out[3] = ((colour_array_in[3] / (float)cs_b_white) * colour_array_out[0]) + ((colour_array_in[3] / (float)cs_b_black) * black_level);
    // Normalise array
    float norm_factor = 3.0/(colour_array_out[1] + colour_array_out[2] + colour_array_out[3]);
    colour_array_out[1] *= norm_factor;
    colour_array_out[2] *= norm_factor;
    colour_array_out[3] *= norm_factor;
    // int sum_black = cs_r_black + cs_g_black + CS_
    // colour_array_out[1] =
}

int Colour::identify_colour_from_calibrated_colour_scores(float * calibrate_colour_array_in)
{
    float scores[8];
    scores[0] = ((calibrate_colour_array_in[1] * 2) * (calibrate_colour_array_in[1] * 2)) / ((calibrate_colour_array_in[2] + calibrate_colour_array_in[3]) * (calibrate_colour_array_in[2] + calibrate_colour_array_in[3]));
    scores[1] = ((calibrate_colour_array_in[1] + calibrate_colour_array_in[2]) * (calibrate_colour_array_in[1] + calibrate_colour_array_in[2])) / (calibrate_colour_array_in[3] * calibrate_colour_array_in[3] * 4);
    scores[2] = ((calibrate_colour_array_in[2] * 2) * (calibrate_colour_array_in[2] * 2)) / ((calibrate_colour_array_in[1] + calibrate_colour_array_in[3]) * (calibrate_colour_array_in[1] + calibrate_colour_array_in[3]));
    scores[3] = ((calibrate_colour_array_in[2] + calibrate_colour_array_in[3]) * (calibrate_colour_array_in[2] + calibrate_colour_array_in[3])) / (calibrate_colour_array_in[1] * calibrate_colour_array_in[1] * 4);
    scores[4] = ((calibrate_colour_array_in[3] * 2) * (calibrate_colour_array_in[3] * 2)) / ((calibrate_colour_array_in[2] + calibrate_colour_array_in[1]) * (calibrate_colour_array_in[2] + calibrate_colour_array_in[1]));
    scores[5] = ((calibrate_colour_array_in[3] + calibrate_colour_array_in[1]) * (calibrate_colour_array_in[3] + calibrate_colour_array_in[1])) / (calibrate_colour_array_in[2] * calibrate_colour_array_in[2] * 4);
    float grey_factor = 1.0 / (1 + ((((calibrate_colour_array_in[1] - 1) * 10) * ((calibrate_colour_array_in[1] - 1) * 10)) * (((calibrate_colour_array_in[2] - 1) * 10) * ((calibrate_colour_array_in[2] - 1) * 10)) * (((calibrate_colour_array_in[3] - 1) * 10) * ((calibrate_colour_array_in[3] - 1) * 10))));
    scores[6] = calibrate_colour_array_in[0] * calibrate_colour_array_in[0] * grey_factor * 4;
    scores[7] = (1-calibrate_colour_array_in[0]) * (1-calibrate_colour_array_in[0]) * grey_factor * 4;
    //pc.printf("R:%1.2f Y:%1.2f G:%1.2f C:%1.2f B:%1.2f M:%1.2f W:%1.2f B:%1.2f G:%1.2f\n\n", scores[0],scores[1],scores[2],scores[3],scores[4],scores[5],scores[6],scores[7],grey_factor);
    float max = 2;
    int max_index = 8;
    for(int i=0; i<8; i++) {
        if(scores[i] > max) {
            max=scores[i];
            max_index=i;
        }
    }
    return max_index;
}

char Colour::IF_check_base_colour_sensor(void)
{
    //Reads the device ID flag of the colour sensor [0xB2]
    //This should equal 0x44 for both TCS34721 (top) and TCS34725 (base) sensors
    //Return a 1 if successful or a 0 otherwise
    char return_value = 0;
    char data[1] = {0x00};
    char command[1] = {0xB2};
    primary_i2c.write(BASE_COLOUR_ADDRESS, command, 1, false);
    primary_i2c.read(BASE_COLOUR_ADDRESS, data, 1, false);
    if(data[0] == 0x44) return_value = 1;
    else psi.debug("Invalid response from colour sensor:%X\n",data[0]);
    return return_value;
}

int Colour::IF_writeSingleRegister( char address, char data )
{
    char tx[2] = { address | 160, data }; //0d160 = 0b10100000
    int ack = primary_i2c.write(BASE_COLOUR_ADDRESS, tx, 2, false);
    return ack;
}

int Colour::IF_writeMultipleRegisters( char address, char* data, int quantity )
{
    char tx[ quantity + 1 ];
    tx[0] = address | 160;
    for ( int i = 1; i <= quantity; i++ ) {
        tx[ i ] = data[ i - 1 ];
    }
    int ack = primary_i2c.write(BASE_COLOUR_ADDRESS, tx, quantity + 1, false);
    return ack;
}

char Colour::IF_readSingleRegister( char address )
{
    char output = 255;
    char command = address | 160; //0d160 = 0b10100000
    primary_i2c.write( BASE_COLOUR_ADDRESS, &command, 1, true );
    primary_i2c.read( BASE_COLOUR_ADDRESS, &output, 1 );
    return output;
}

int Colour::IF_readMultipleRegisters( char address, char* output, int quantity )
{
    char command = address | 160; //0d160 = 0b10100000
    primary_i2c.write(BASE_COLOUR_ADDRESS, &command, 1, true );
    int ack = primary_i2c.read( BASE_COLOUR_ADDRESS, output, quantity );
    return ack;
}

char cl_description[90];
const char * Colour::IF_get_calibration_values_string()
{
    sprintf(cl_description,"[BLACK C:%4d R:%4d G:%4d B:%4d WHITE C:%4d R:%4d G%4d B:%4d]",cs_c_black,cs_r_black,cs_g_black,cs_b_black,cs_c_white,cs_r_white,cs_g_white,cs_b_white);
    return cl_description;
}