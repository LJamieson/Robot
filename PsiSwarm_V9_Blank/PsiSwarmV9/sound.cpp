/* University of York Robotics Laboratory PsiSwarm Library: Audio Driver using PIC coprocessor Source File
 *
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: sound.cpp [formerly pic.cpp]
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


void Sound::play_audio_string(char * tune)
{
    char length = strlen(tune);
    play_tune(tune,length);
}

void Sound::play_tune(char * tune, char length)
{
    char to_send [length+3];
    char start_array[2];
    start_array [0] = 'S';
    start_array [1] = length;
    strcpy(to_send,start_array);
    strncat(to_send,tune,length);
    psi.debug(to_send);
    primary_i2c.write(PIC_ADDRESS,to_send,length+2,false);
}


char Sound::IF_check_pic_firmware()
{
    char buffer[6];
    buffer[0] = 0;
    primary_i2c.write(PIC_ADDRESS,"I",1,false);
    wait(0.1);
    primary_i2c.read(PIC_ADDRESS,buffer,6);
    psi.debug(buffer);
    if(buffer[0] != 'F' || buffer[1] != 'W') {
        psi.debug("WARNING:  Cannot read information from PIC microcontroller");
        return 1;
    }
    psi.debug(buffer);
    return 0;
}