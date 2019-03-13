/* University of York Robotics Laboratory PsiSwarm Library: Display Driver Source File
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: display.cpp
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * 
 * James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
 * Driver for the Midas 16x2 I2C LCD Display (MCCOG21605x6W) LCD
 * [Farnell part 2218942 or 2063206]
 *
 */ 
 
#include "psiswarm.h"

Timeout init_timeout;
Timeout backlight_timeout;
Timeout debug_timeout;
int backlight_on_time;
int backlight_off_time;
char backlight_step;
char multipage[200];
char multipage_length = 0;
char preserve_line_1 [17];
char preserve_line_2 [17];
char c_row=0;
char c_column=0;
char p_row;
char p_column;


Display::Display(PinName sda, PinName scl, PinName reset, PinName backlight) :  Stream("display"), _i2c(sda,scl), _reset(reset), _backlight(backlight)  {
}

Display::Display() :  Stream("display"), _i2c(p28,p27), _reset(p29), _backlight(p30)  {
}

int Display::i2c_message(char byte){
   char bytes [2];
   bytes[0]=0x80;
   bytes[1]=byte;
   int ret=_i2c.write(LCD_ADDRESS,bytes,2);  
   wait(0.01);
   return ret;
}

int Display::disp_putc(int c){
   char message [2];
   message[0]=0x40;
   message[1]=c;
   _i2c.write(LCD_ADDRESS,message,2);
   wait(0.01);
   return c;
}
void Display::init_display_start(){
   display_on = 1;
   set_backlight_brightness(1);
   cursor_on = 0;
   blink_on  = 0;
    
   _reset=1;
   wait(0.02);
   //Set reset low
   _reset=0;
   wait(0.001);
   _reset=1;
   wait(0.03);
   i2c_message(0x38); 
   i2c_message(0x39); 
   i2c_message(0x14); 
   i2c_message(0x74); 
   i2c_message(0x54); 
   i2c_message(0x6F); 
   _set_display();
   clear_display(); 
   char psis[17];
   for(int i=0;i<16;i++){
       psis[i]=0x1D;
   }
   set_position(0,0);
   write_string(psis,16);
   set_position(1,0);
   write_string(psis,16);
}

void Display::init_display_end(char mode){
   clear_display();
    if(mode == 0) {
        set_position(0,0);
        write_string("  YORK ROBOTICS");
        set_position(1,0);
        write_string("   LABORATORY");
   init_timeout.attach(this,&Display::post_init,0.25);}
   else {      
   set_position(0,0);
   write_string("Hold button to");
   set_position(1,0);
   write_string("launch demo code");
    }
} 

void Display::show_switch_state(char switch_state){
    switch(switch_state){
        /// Switch_state = 1 if up is pressed, 2 if down is pressed, 4 if left is pressed, 8 if right is pressed and 16 if the center button is pressed
        case 0: write_string("REL   "); break;
        case 1: write_string("UP    "); break;
        case 2: write_string("DOWN  "); break;
        case 4: write_string("LEFT  "); break;
        case 5: write_string("UP-L  "); break;
        case 6: write_string("DN-L  "); break;
        case 8: write_string("RIGHT "); break;
        case 9: write_string("UP-R  "); break;
        case 10: write_string("DN-R  "); break;
        case 16: write_string("PRESS"); break;
        case 17: write_string("UP   *"); break;
        case 18: write_string("DOWN *"); break;
        case 20: write_string("LEFT *"); break;
        case 21: write_string("UP-L *"); break;
        case 22: write_string("DN-L *"); break;
        case 24: write_string("RIGHT*"); break;
        case 25: write_string("UP-R *"); break;
        case 26: write_string("DN-R *"); break;
    }    
}

void Display::post_init(){
    clear_display();
    home();
    write_string("PSI SWARM ROBOT");
    set_position(1,0);
    char line [17];
    sprintf(line,"VERSION %1.2f", SOFTWARE_VERSION_CODE  );
    set_position(1,0);
    write_string(line);
    init_timeout.attach(this,&Display::post_post_init,0.25);
}

void Display::post_post_init(){
    clear_display();
    home();
}

void Display::write_string(char * message){
   size_t length = strlen(message);
   if (length > 16) length = 16;
   char to_send [length+1];
   to_send[0]=0x40;
   for(int i=0;i<length;i++){
     to_send[i+1] = message[i];
   }
   _i2c.write(LCD_ADDRESS,to_send,length+1);
   // Add to saved buffer
   int count = 0;
   for(int i=c_column;i<16;i++){
       if(count < length){
            if(c_row == 0) preserve_line_1[i] = message[count];
            else preserve_line_2[i] = message[count];  
       }
       count++;
   }
   c_column+=length;
   if(c_column>15) c_column=15;
}


void Display::write_string(char * message, char length){
   char to_send [length+1];
   to_send[0]=0x40;
   for(int i=0;i<length;i++){
     to_send[i+1] = message[i];
   }
   _i2c.write(LCD_ADDRESS,to_send,length+1);
   // Add to saved buffer
   int count = 0;
   for(int i=c_column;i<16;i++){
       if(count < length){
            if(c_row == 0) preserve_line_1[i] = message[count];
            else preserve_line_2[i] = message[count];  
       }
       count++;
   }
   c_column+=length;
   if(c_column>15) c_column=15;
}

void Display::set_position(char row, char column){
  if(row < 2 && column < 16){
    char pos = 128 +((row * 64)+column);
    i2c_message(pos);
    c_row= row;
    c_column = column;
  }
}

void Display::set_cursor(char enable){
  cursor_on=enable;
  _set_display();
}

void Display::set_blink(char enable){
  blink_on=enable;
    _set_display();
}

void Display::set_display(char enable){
  display_on=enable;
    _set_display();
}

void Display::set_backlight_brightness(float brightness){
    if(brightness > 1) brightness = 0;
    if(brightness < 0) brightness = 0;
    backlight_brightness = brightness;
    if(backlight_brightness == 1) {
        backlight_timeout.detach();   
        _backlight = 1;
    }else{
        if(backlight_brightness == 0){
            backlight_timeout.detach();
            _backlight = 0;   
        }   else {
            backlight_on_time = (int) (10000.0f * backlight_brightness);
            backlight_off_time = 10000 - backlight_on_time;
            backlight_step = 0;
            _backlight = 0;
            backlight_timeout.attach_us(this,&Display::IF_backlight_toggle,backlight_off_time);
        }
    }
}

void Display::IF_backlight_toggle(){
    if(backlight_step == 0){
        _backlight = 1;
        backlight_step = 1;
        backlight_timeout.attach_us(this,&Display::IF_backlight_toggle,backlight_on_time);
    }   else {
        _backlight = 0;
        backlight_step = 0;
        backlight_timeout.attach_us(this,&Display::IF_backlight_toggle,backlight_off_time);
    }
}
void Display::clear_display(){
  for(int i=0;i<16;i++){
      preserve_line_1[i] = 0x20;
      preserve_line_2[i] = 0x20;
  }
  i2c_message(0x01);
}

void Display::home(){
  c_row = 0;
  c_column = 0;
  i2c_message(0x02);
}

void Display::debug_page(char * message, char length){
    p_row=c_row;
    p_column=c_column;
    i2c_message(0x01);
    home();
    char multipage_mode = 0;
    char line_1[18];
    char line_2[18];
    line_1[0]=0x40;
    line_2[0]=0x40;
    if(length > 16){
        strncpy(line_1+1, message, 16);
        char f_length = length - 16;
        if(f_length > 16) {
            f_length = 16;
            multipage_mode = 1;
        }
        strncpy(line_2+1, message+16, f_length);
        line_1[17]=0;
        line_2[f_length+1]=0;
        _i2c.write(LCD_ADDRESS,line_1,17);
        set_position(1,0);
        _i2c.write(LCD_ADDRESS,line_2,f_length+1);
    } else {
        strncpy(line_1+1, message, length);
        _i2c.write(LCD_ADDRESS,line_1,length+1);
        }
    if(multipage_mode == 1){
        strncpy(multipage, message + 32, length - 32);
        multipage_length = length - 32;
        debug_timeout.attach(this,&Display::IF_debug_multipage,PAGE_TIME);
    } else debug_timeout.attach(this,&Display::IF_restore_page,CLEAR_TIME);
}

void Display::IF_restore_page(){
    i2c_message(0x01);
    home();
    char line_1[17];
    char line_2[17];
    line_1[0]=0x40;
    line_2[0]=0x40;
    strncpy(line_1+1, preserve_line_1, 16);
    strncpy(line_2+1, preserve_line_2, 16);
    _i2c.write(LCD_ADDRESS,line_1,17);
    set_position(1,0);
    _i2c.write(LCD_ADDRESS,line_2,17);
    set_position(p_row,p_column);
}

void Display::IF_debug_multipage(){
    i2c_message(0x01);
    home();
    char multipage_mode = 0;
    char line_1[18];
    char line_2[18];
    line_1[0]=0x40;
    line_2[0]=0x40;
    if(multipage_length > 16){
        strncpy(line_1+1, multipage, 16);
        char f_length = multipage_length - 16;
        if(f_length > 16) {
            f_length = 16;
            multipage_mode = 1;
        }
        strncpy(line_2+1, multipage+16, f_length);
        line_1[17]=0;
        line_2[f_length+1]=0;
       _i2c.write(LCD_ADDRESS,line_1,17);
        set_position(1,0);
        _i2c.write(LCD_ADDRESS,line_2,f_length+1);
    } else  {
        strncpy(line_1+1, multipage, multipage_length);
        _i2c.write(LCD_ADDRESS,line_1,multipage_length+1);
        }
    if(multipage_mode == 1){
        char temp[200];
        strncpy(temp, multipage + 32, multipage_length - 32);
        multipage_length -= 32;
        strncpy(multipage, temp, multipage_length);
        debug_timeout.attach(this,&Display::IF_debug_multipage,PAGE_TIME);
    }else debug_timeout.attach(this,&Display::IF_restore_page,CLEAR_TIME);
}

void Display::_set_display(){
    char mode = 8;
    if(display_on>0) mode += 4;
    if(cursor_on>0) mode += 2;
    if(blink_on>0) mode ++;
    i2c_message(mode);
}


int Display::_putc (int c) {
    putc(c);
    return(c);
}

int Display::_getc (void) {
    char r = 0;
    return(r);
}
