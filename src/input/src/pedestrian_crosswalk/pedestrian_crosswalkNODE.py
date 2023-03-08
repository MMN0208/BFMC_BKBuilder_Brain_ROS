#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import json

import rospy

from std_msgs.msg import Byte

class pedestrian_crosswalkNODE():
    def __init__(self):
        """listener class. 
        
        Class used for running port listener algorithm 
        """                
        rospy.init_node('pedestrian_crosswalkNODE', anonymous=False)
        
        # BNO publisher object
        self.pedestrian_crosswalk = rospy.Publisher("/automobile/pedestrian_crosswalk", Byte, queue_size=1)

    #================================ RUN ========================================
    def run(self):
        """ Method for running listener algorithm.
        """
        rospy.loginfo("starting pedestrian_crosswalkNODE")
        self._getting()

    #================================ GETTING ========================================
    def _getting(self): 
        # Listen for incomming broadcast messages
        while not rospy.is_shutdown():
            # Wait for data
            try:
                # car wait when pedestrian present on (middle lane) or (right lane with crosswalk_detect).
                # car run when there is no pedestrian or pedestrian on the left lane ()
                # pedestrian crossed = pedestrian passed to left land
                # ID = 0: none 1: right with crosswalk_detect 2: middle 3: left 4: right without crosswalk_detect
                ID = 2
                self.pedestrian_crosswalk.publish(ID)

            except Exception as e:
                if str(e) !="timed out":
                    print("Receiving data failed with error: " + str(e))
                
if __name__ == "__main__":
    pedNOD = pedestrian_crosswalkNODE()
    pedNOD.run()