import numpy as np


def add_noise(data: list):
    return data + np.random.normal(0., 1., len(data))

def save_csv():
    pass