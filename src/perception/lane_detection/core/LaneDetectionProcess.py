import numpy as np
from .camera import Camera
from threading import Thread
import cv2 as cv

from src.templates.workerprocess import WorkerProcess

class LaneDetectionProcess(WorkerProcess):

    def __init__(self, inPs, outPs):

        super(LaneDetectionProcess, self).__init__(inPs, outPs)
        self.outPs = outPs
        self.inPs = inPs
        self.camera = Camera()
        self.threads = list()
    def _init_threads(self):

        if self._blocker.is_set():
            return

        sendBirdEyeThread = Thread(name = "sendBirdEye", target=self._send_threads, args = (self.inPs[0],))
        sendBirdEyeThread.daemon = True
        self.threads.append(sendBirdEyeThread)

    def run(self):
        
        super(LaneDetectionProcess, self).run()

    
    def _send_threads(self, inP):

        """
        Keys:
            birdeye_img: frame of bird eye view -> Shape (360, 640)
            inverse_transform: POV to convert into the original view
            transfored_view: POV to convert into bird view
            gray: gray scale of original image
            thresh: segmentation r.s.t the masking with threshold
            canny: edges

        """
        while True:
            timestamp, frame = inP.recv()
            # frame = cv.resize(frame, (144,144))
            processing_result = self.camera.laneDetector.processor.process(frame)
            #print(processing_result['thresh'].shape)
            for outP in self.outPs:
                outP.send([processing_result])
    

         
