/***********************************************************************
**  ██████╗ ███████╗██╗███████╗██╗    ██╗ █████╗ ██████╗ ███╗   ███╗  **
**  ██╔══██╗██╔════╝██║██╔════╝██║    ██║██╔══██╗██╔══██╗████╗ ████║  **
**  ██████╔╝███████╗██║███████╗██║ █╗ ██║███████║██████╔╝██╔████╔██║  **
**  ██╔═══╝ ╚════██║██║╚════██║██║███╗██║██╔══██║██╔══██╗██║╚██╔╝██║  **
**  ██║     ███████║██║███████║╚███╔███╔╝██║  ██║██║  ██║██║ ╚═╝ ██║  **
**  ╚═╝     ╚══════╝╚═╝╚══════╝ ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝  **
************************************************************************
** Copyright 2016 University of York                                  **
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

/// PsiSwarm Blank Example Code
/// Version 0.8
/// James Hilder, Alan Millard, Alexander Horsfield, Homero Elizondo, Jon Timmis
/// University of York

#ifndef MAIN_H
#define MAIN_H

#include "psiswarm.h"
#include "cstdlib"

int main(void);
void user_code_setup(void);
void user_code_loop(void);
void handle_switch_event(char switch_state); 
void handle_user_serial_message(char * message, char length, char interface);

#endif