import time
import numpy as np


def measure_time(func):

    def inner(*args, **kwargs):

        begin = time.time()

        func(*args, **kwargs)

        end = time.time()

        print("Time elapsed = {}".format(end - begin))