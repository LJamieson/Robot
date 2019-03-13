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
 * File: sound.h
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
 
 
#ifndef SOUND_H
#define SOUND_H

/**
 * Sound class
 * Functions that generate audio tones using the sound module on the PIC coprocessor, where used
 *
*/
class Sound
{
public:

 /** Play a tune defined by the given null terminated string
  * @param tune - The tune to play
  */
void play_audio_string(char * tune);

 /** Play a tune defined by the given string
  * @param tune - The tune to play
  * @param length - The number of characters in the string
  */
void play_tune(char * tune, char length);
char IF_check_pic_firmware(void);
};
#endif
