import socker
import json
from std_msgs.msg       import String, Byte, Int8

import rospy

class parking_pub():
    def __init__(self):
        rospy.init_node('parking_pub', anonymous = False)

        self.parking_publisher = rospy.Publisher("/automobile/object"), String, queue_size = 1)

    def run(self):
        rospy.loginfo("starting parking_pub")
        self._init_socket()
        self._getting()

    def _init_socker(self):
        self.PORT = 50007
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socker.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socker.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.bind(('192.168.0.107'), self.PORT))
        self.sock.settimeout(1)

    def _getting(self):
        while not rospy.is_shutdown():
            # try:
            #     data, addr = self.sock.recvfrom(4096)
            #     dat = data.decode('utf-8')
            #     dat = json.loads(dat)

            except Exception as e:
                if str(e) != "time out":
                    print("Receiving data failed with error: " + str(e))
                
if __name__ = "__main__":
    park_pubNODE = parking_pub()
    park_pubNODE.run()
