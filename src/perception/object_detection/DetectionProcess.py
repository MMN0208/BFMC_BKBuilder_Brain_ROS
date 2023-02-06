import socket
import struct
import time
import numpy as np

from threading import Thread

import os
import sys
import argparse
import logging
import time
from pathlib import Path
import glob
import json

import numpy as np
from tqdm import tqdm
import cv2
import yaml

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from src.utils.detection.edgetpumodel import * 
from src.utils.detection.edgetpumodel.utils import plot_one_box, Colors, get_image_tensor

from src.templates.workerprocess import WorkerProcess

class DetectionProcess(WorkerProcess):
    # ===================================== INIT =========================================
    def __init__(self, inPs, outPs):
        """Process used for sending images over the network to a targeted IP via UDP protocol 
        (no feedback required). The image is compressed before sending it. 

        Used for visualizing your raspicam images on remote PC.
        
        Parameters
        ----------
        inPs : list(Pipe) 
            List of input pipes, only the first pipe is used to transfer the captured frames. 
        outPs : list(Pipe) 
            List of output pipes (not used at the moment)
        """
        super(DetectionProcess,self).__init__( inPs, outPs)
        self.model_path = "src/utils/detection/weights/traffic.tflite"
        self.names = "src/utils/detection/data.yaml"
        self.conf_thresh = 0.5
        self.iou_thresh = 0.65
        self.device = 0
        self.model = None 
        self.colors = Colors()
        #SENDING
        self.outPs        =   outPs
        #print(f"YOLOv5 Image Size {self.model.get_image_size()}")
    # ===================================== RUN ==========================================
    def run(self):
        """Apply the initializing methods and start the threads.
        """
        super(DetectionProcess,self).run()
        
    # ===================================== INIT THREADS =================================
    def _init_threads(self):
        """Initialize the sending thread.
        """
        if self._blocker.is_set():
            return
        streamTh = Thread(name='DetectionThread',target = self._send_thread, args= (self.inPs[0], ))
        streamTh.daemon = True
        self.threads.append(streamTh)
      
    # ===================================== SEND THREAD ==================================
    def _send_thread(self, inP):
        """Sending the frames received thought the input pipe to remote client by using the created socket connection. 
        
        Parameters
        ----------
        inP : Pipe
            Input pipe to read the frames from CameraProcess or CameraSpooferProcess. 
        """
        self.model = EdgeTPUModel(self.model_path, self.names, conf_thresh=self.conf_thresh, iou_thresh=self.iou_thresh)
        logging.disable(logging.CRITICAL)
        logger.disabled = False
        while True:
            try:
                stamps, image = inP.recv()
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                output_image = image
                #TODO
                full_image, net_image, pad = get_image_tensor(image, 640)
                pred = self.model.forward(net_image)
                #print(f"DetectionProcess{net_image.shape}")
                det = self.model.process_predictions(pred[0], full_image, pad)
                #print(a)
                for *xyxy, conf, cls in reversed(det):
                    #Process Detections
                    c = int(cls)  # integer class
                    label = f'{self.names[c]} {conf:.2f}'
                    output_image = plot_one_box(xyxy, output_image, label=label, color=self.colors(c, True))
                    #pass
                tinference, tnms = self.model.get_last_inference_time()
                print("Frame done in {}".format(tinference+tnms))
                
                stamp = time.time()
                #SEND DETECTIONS
                for outP in self.outPs:
                    outP.send([[stamp], output_image])

            except Exception as e:
                print("Detection failed to detect images:",e,"\n")
                pass