import subprocess
import platform
import copy
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import caffe
import caffe.draw
import google.protobuf



caffe.set_device(0)
caffe.set_mode_gpu()
solver = caffe.get_solver('solverYAxis.prototxt')
solver.solve()
