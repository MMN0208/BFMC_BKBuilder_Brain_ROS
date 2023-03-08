import time
import numpy as np


def measure_time(func):

    def inner(*args, **kwargs):

        begin = time.time()

        func(*args, **kwargs)

        end = time.time()

        print("Time elapsed = {}".format(end - begin))
    return inner
@measure_time
def better(mat):

    indices = np.argwhere(mat >= 240)
    sum_ = indices.sum(axis = 0)
    
    count = indices.shape[0]
    center_x = sum_[1] / count
    center_y = sum_[0] / count

    print(center_x, center_y)

if __name__ == '__main__':

    mat = np.random.randint(0, 255, (480, 640))

    better(mat)
    