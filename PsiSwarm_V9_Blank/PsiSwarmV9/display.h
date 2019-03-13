/* University of York Robotics Laboratory PsiSwarm Library: Display Driver Header File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: display.h
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
 *
 * Driver for the Midas 16x2 I2C LCD Display (MCCOG21605x6W) LCD
 *
 * Farnell part 2218942 or 2063206
 *
 */


#ifndef DISPLAY_H
#define DISPLAY_H

#define PAGE_TIME 0.4
#define CLEAR_TIME 0.8

/**
 * Display class
 * Functions for use with the Midas 16x2 I2C LCD Display (MCCOG21605x6W) LCD
 * Farnell part 2218942 or 2063206
 *
 * Example:
 * @code
 * #include "psiswarm.h"
 *
 * int main() {
 *     init();
 *     display.clear_display;       //Clears display
 *     display.set_position(0,2);   //Set cursor to row 0 column 2
 *     display.write_string("YORK ROBOTICS");
 *     display.set_position(1,3);   //Set cursor to row 1 column 3
 *     display.write_string("LABORATORY");
 * }
 * @endcode
*/
class Display : public Stream
{

// Public Functions

public:

    /** Create the LCD Display object connected to the default pins
     * (sda = p28, scl = p27, reset = p29, backlight = p30)
     */
    Display();

    /** Create the LCD Display object connected to specific pins
     *
     * @param sda pin   - default is p28
     * @param scl pin   - default is p27
     * @param reset pin - default is p29
     * @param backlight pin - default is p30
     */
    Display(PinName sda, PinName scl, PinName reset, PinName backlight);

    /** Clear the display
    */
    void clear_display(void);

    /** Set cursor to home position
    */
    void home(void);

    /** Print string message
    * @param message - The null-terminated message to print
    */
    void write_string(char * message);

    /** Print string message of given length
    * @param message - The message to print
    * @param length - The number of characters to display
    */
    void write_string(char * message, char length);

    /** Set the row and column of cursor position
    * @param row - The row of the display to set the cursor to (either 0 or 1)
    * @param column - The column of the display to set the cursor to (range 0 to 15)
    */
    void set_position(char row, char column);

    /** Enable or disable cursor
    * @param enable - Set to 1 to enable the cursor visibility
    */
    void set_cursor(char enable);

    /** Enable or disable cursor blink
    * @param enable - Set to 1 to enable the cursor blinking mode
    */
    void set_blink(char enable);

    /** Enable or disable display
    * @param enable - Set to 1 to enable the display output
    */
    void set_display(char enable);

    /** Set the brightness of the backlight
    * @param brightness - Sets the brightness of the display (range 0.0 to 1.0)
    */
    void set_backlight_brightness(float brightness);

    /** Display the switch state at the current cursor position
    * @param switch_state - The value of the cursor switch (range 0 to 31)
    */
    void show_switch_state(char switch_state);
    
    // Special function for when debug messages are sent to display
    void debug_page(char * message, char length);

    // Internal function used to restore display after debug messages
    void IF_restore_page(void);

    // Internal function used to show multi-page debug messages
    void IF_debug_multipage(void);

    // Internal function used to toggle backlight
    void IF_backlight_toggle(void);

    //Parts of initialisation routine
    void post_init(void);
    void post_post_init(void);

    // Send a 1-byte control message to the display
    int i2c_message(char byte);

    // Default initialisation sequence for the display (start of boot)
    void init_display_start();

    // Default initialisation sequence for the display (end of boot)
    void init_display_end(char mode);

    int disp_putc(int c);


private :

    I2C _i2c;
    DigitalOut _reset;
    DigitalOut _backlight;

    char display_on;
    char cursor_on;
    char blink_on;

    void _set_display();

    virtual int _putc(int c);
    virtual int _getc();

};

#endif // DISPLAY_H