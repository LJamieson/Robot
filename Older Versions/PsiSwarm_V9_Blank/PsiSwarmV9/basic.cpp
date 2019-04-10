/* University of York Robotics Laboratory PsiSwarm Library: Psi-BASIC Interpretter Code
 * 
 * Copyright 2017 University of York
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and limitations under the License.
 *
 * File: basic.cpp
 *
 * Dept. Electronics & Computer Science, University of York
 * James Hilder, Alexander Horsfield, Alan Millard, Homero Elizondo, Jon Timmis
 *
 * PsiSwarm Library Version: 0.9
 *
 * June 2017
 *
 */
 
 // TODO:  This is largely unimplemented at present - will be based on AH student project
 
 #include "psiswarm.h"
 
 LocalFileSystem local("local");
 
 void Basic::read_list_of_file_names()
 {
    DIR *dp;
    struct dirent *dirp;
    dp = opendir("/local");
    if(dp == NULL) pc.printf("- File handling error: Failed to open directory\n");
    psi.debug("- Reading FLASH storage for PsiBasic files\n");
    //read all files in MBED root directory and add matching file names in current directory into filename vector
    while((dirp = readdir(dp)) != NULL) {
        string filename = (string) dirp->d_name;
         if (filename.compare(filename.size()-4,4,".PSI") == 0)
            {
                psi_basic_file_count ++ ;
                psi.debug("- Found file: %s\n",filename.c_str());
                basic_filenames.push_back(filename);
            }
    }
    closedir(dp);
 }

