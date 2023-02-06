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

from edgetpumodel import *

# from edgetpumodel import EdgeTPUModel
# from utils import resize_and_pad, get_image_tensor, save_one_json, coco80_to_coco91_class


def run(model, quiet=False, stream=True, image_t=None, bench_speed=False, bench_image=False):
    if quiet:
        logging.disable(logging.CRITICAL)
        logger.disabled = True
    
    if stream and image_t:
        logger.error("Please select either an input image or a stream")
        exit(1)
    
    input_size = model.get_image_size()

    x = (255*np.random.random((3,*input_size))).astype(np.uint8)
    model.forward(x)

    conf_thresh = 0.25
    iou_thresh = 0.45
    classes = None
    agnostic_nms = False
    max_det = 1000

    if bench_speed:
        logger.info("Performing test run")
        n_runs = 100
        
        
        inference_times = []
        nms_times = []
        total_times = []
        
        for i in tqdm(range(n_runs)):
            x = (255*np.random.random((3,*input_size))).astype(np.float32)
            
            pred = model.forward(x)
            tinference, tnms = model.get_last_inference_time()
            
            inference_times.append(tinference)
            nms_times.append(tnms)
            total_times.append(tinference + tnms)
            
        inference_times = np.array(inference_times)
        nms_times = np.array(nms_times)
        total_times = np.array(total_times)
            
        logger.info("Inference time (EdgeTPU): {:1.2f} +- {:1.2f} ms".format(inference_times.mean()/1e-3, inference_times.std()/1e-3))
        logger.info("NMS time (CPU): {:1.2f} +- {:1.2f} ms".format(nms_times.mean()/1e-3, nms_times.std()/1e-3))
        fps = 1.0/total_times.mean()
        logger.info("Mean FPS: {:1.2f}".format(fps))

    elif bench_image:
        logger.info("Testing on Zidane image")
        model.predict("./data/images/zidane.jpg")
    
    elif image_t is not None:
        logger.info("Testing on user image: {}".format(image_t))
        model.predict(image_t)
        
    elif stream:
        logger.info("Opening stream on device: {}".format(device))
        
        cam = cv2.VideoCapture(device)
        
        while True:
          try:
            res, image = cam.read()
            
            if res is False:
                logger.error("Empty image received")
                break
            else:
                full_image, net_image, pad = get_image_tensor(image, input_size[0])
                pred = model.forward(net_image)
                
                a = model.process_predictions(pred[0], full_image, pad)
                print(a)
                tinference, tnms = model.get_last_inference_time()
                logger.info("Frame done in {}".format(tinference+tnms))
          except KeyboardInterrupt:
            break
          
        cam.release()

if __name__ == "__main__":
    model_path = "weights/traffic.tflite"
    names = "data.yaml"
    conf_thresh = 0.5
    iou_thresh = 0.65
    device = 0
    # parser = argparse.ArgumentParser("EdgeTPU test runner")
    # parser.add_argument("--model", "-m", help="weights file", required=True)
    # parser.add_argument("--bench_speed", action='store_true', help="run speed test on dummy data")
    # parser.add_argument("--bench_image", action='store_true', help="run detection test")
    # parser.add_argument("--conf_thresh", type=float, default=0.25, help="model confidence threshold")
    # parser.add_argument("--iou_thresh", type=float, default=0.45, help="NMS IOU threshold")
    # parser.add_argument("--names", type=str, default='data/coco.yaml', help="Names file")
    # parser.add_argument("--image", "-i", type=str, help="Image file to run detection on")
    # parser.add_argument("--device", type=int, default=0, help="Image capture device to run live detection")
    # parser.add_argument("--stream", action='store_true', help="Process a stream")
    # parser.add_argument("--bench_coco", action='store_true', help="Process a stream")
    # parser.add_argument("--coco_path", type=str, help="Path to COCO 2017 Val folder")
    # parser.add_argument("--quiet","-q", action='store_true', help="Disable logging (except errors)")
        
    # args = parser.parse_args()
    model = EdgeTPUModel(model_path, names, conf_thresh=conf_thresh, iou_thresh=iou_thresh)
    run(model)

            
        

    

