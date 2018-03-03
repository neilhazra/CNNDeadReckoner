import subprocess
import platform
import copy
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import caffe
import caffe.draw
import google.protobuf


net = caffe.Net('conv.prototxt', caffe.TEST)
