/* University of York Robotics Laboratory PsiSwarm Library: Psi-BASIC Interpretter Code Header File
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 * 
 * File: basic.h
 *
 * (C) Dept. Electronics & Computer Science, University of York
 * James Hilder, Alexander Horsfield, Alan Millard, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
 */
 
 // TODO:  This is largely unimplemented at present - will be based on AH student project

#ifndef BASIC_H
#define BASIC_H

/**
 *  The Basic class contains the functions for the Psi Basic interpreter and file-handling
*/
class Basic{
    public:
    /**
     * Read the list of Psi Basic filenames from the MBED Flash memory 
    */
    void read_list_of_file_names(void);
};
#endif