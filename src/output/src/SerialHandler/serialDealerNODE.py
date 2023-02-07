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

import socket

import rospy

import time

from std_msgs.msg import String

class serialDealerNODE():
    def __init__(self):
        """It forwards the control messages received from socket to the serial handling node. 
        """
        
        rospy.init_node('serialDealerNODE', anonymous=False)
        
        # Command pubSSlisher object
        #self.command_publisher = rospy.Publisher("/automobile/command", String, queue_size=1)
        self.command_publisher = rospy.Publisher("/automobile/perception",String, queue_size=1)
    
     # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads
        """
        rospy.loginfo("starting serialDealerNODE")
        self._init_socket()
        self._read_stream()

    # ===================================== INIT SOCKET ==================================
    def _init_socket(self):
        """Initialize the communication socket server.
        """
        self.port       =   12244
        self.serverIp   =   '192.168.0.107'
        
        self.server_socket = socket.socket(
                                    family  = socket.AF_INET, 
                                    type    = socket.SOCK_DGRAM
                                )
        self.server_socket.bind((self.serverIp, self.port))
        self.server_socket.settimeout(1)
        
    def _read_stream(self):
        """Receive the message and forwards them to the Serial Handler Node. 
        """
        
        while not rospy.is_shutdown():
            try:
                #print("hello")
                bts, addr = self.server_socket.recvfrom(1024)
                #print(bts)
                command   =  bts.decode()
                #command = String("hello")
                self.command_publisher.publish(command)
                #stime.sleep(2)
            except:
                pass
        else:
            self.server_socket.close()
            
            
if __name__ == "__main__":
    serDealerNod = serialDealerNODE()
    serDealerNod.run()
