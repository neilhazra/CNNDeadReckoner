import subprocess
import platform
import copy
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import caffe
import caffe.draw
import google.protobuf


#net = caffe.Net('convXAxis.prototxt', caffe.TEST)
caffe.set_device(0)
caffe.set_mode_gpu()
solver = caffe.get_solver('solverXAxis.prototxt')
solver.solve()
