/***********************************************************************
**  ██████╗ ███████╗██╗███████╗██╗    ██╗ █████╗ ██████╗ ███╗   ███╗  **
**  ██╔══██╗██╔════╝██║██╔════╝██║    ██║██╔══██╗██╔══██╗████╗ ████║  **
**  ██████╔╝███████╗██║███████╗██║ █╗ ██║███████║██████╔╝██╔████╔██║  **
**  ██╔═══╝ ╚════██║██║╚════██║██║███╗██║██╔══██║██╔══██╗██║╚██╔╝██║  **
**  ██║     ███████║██║███████║╚███╔███╔╝██║  ██║██║  ██║██║ ╚═╝ ██║  **
**  ╚═╝     ╚══════╝╚═╝╚══════╝ ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝  **
************************************************************************
** Copyright 2017 University of York - See notice at end of file      **
***********************************************************************/

/// PsiSwarm C++ Blank Example Code - Version 0.9 - June 2017
/// James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis

/// Include main.h - this includes psiswarm.h all the other necessary core files
#include "main.h"
#include "cstdlib"

Psiswarm psi;
string command;
char * program_name = "Blank";
char * author_name  = "YRL";
char * version_name = "0.90";

///Place user code here that should be run after initialisation but before the main loop
void user_code_setup()
{
    wait(0.8); // The display is still updating from init so wait a short while
    display.clear_display();
    display.set_position(0,0);
    animations.set_colour(1);
    display.write_string(" [  V2.8   ]");   
}

///User code loop:  This is where user code should go; it is run as an infinite loop
void user_code_loop()
{
    // Replace this code with your main loop
    //
    //
    if (command.find("forward") != string::npos){
        char buffc[50]="";// holds the char for the numbers after the comands
        int countc = 7;// counter for the and of the commnd
        int countb = 0;//counter for the number
        while(command[countc]&&countb<8){// takes the numbers after the command
            buffc[countb] = command[countc];
            countc++;
            countb++;
            }
        float vall= atof(buffc);//converts the string of numbers to float
        command="";// resets the command to null
        if(vall<=1&&vall>0){//evaluates the values to be in range
            motors.forward(vall);//execute the function
        }else{
            printf("Invalid motor.forward value!(%f)",vall);// print worning mesage
        }
        }
    if(command=="brake"){
    motors.brake();
    command="";
    }
    animations.led_run1();
    wait(1.2);
}

/// Code goes here to handle what should happen when the user switch is pressed
void handle_switch_event(char switch_state)
{
    /// Switch_state = 1 if up is pressed, 2 if down is pressed, 4 if left is pressed, 8 if right is pressed and 16 if the center button is pressed
    /// NB For maximum compatability it is recommended to minimise reliance on center button press
}

void handle_user_serial_message(char * message, char length, char interface)
{
    // This is where user code for handling a (non-system) serial message should go
    //
    // message = pointer to message char array
    // length = length of message
    // interface = 0 for PC serial connection, 1 for Bluetooth
    printf("final input : %s\n",message);//testing perpose
    command = message;
}

/// The main routine: it is recommended to leave this function alone and add user code to the above functions
int main()
{
    psi.init();             ///psi.init() in psiswarm.cpp sets up the robot
    user_code_setup();      ///run user code setup block
    user_code_running = 1;  ///nb. user code can be paused by external commands sent from PC\BT interfaces
    while(1) {
        if(user_code_running == 1) user_code_loop();   ///run user code
        else wait_us(1000);
    }
}


/***********************************************************************
** Copyright 2017 University of York                                  **
**                                                                    **
** Licensed under the Apache License, Version 2.0 (the "License")     **
** You may not use this file except in compliance with the License.   **
** You may obtain a copy of the License at                            **
** http://www.apache.org/licenses/LICENSE-2.0   Unless required by    **
** applicable law or agreed to in writing, software distributed under **
** under the License is distributed on an "AS IS" BASIS WITHOUT       **
** WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   **
** See the License for the specific language governing permissions    **
** and limitations under the License.                                 **
***********************************************************************/
