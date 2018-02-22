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
def timehelper():
	inputArray = np.asarray([[np.transpose(input)]]);
	net.blobs['data'].data[...] = inputArray
	output = net.forward()
net = caffe.Net('deploy.prototxt','models/model3_train_0422/solver_iter_500.caffemodel',caffe.TEST)
input = [ [0,0,0,0,0,0],
		  [0,0,0,0,0,0],
		  [0,0,0,0,0,0],
          [0,0,0,0,0,0],
          [0,0,0,0,0,0],
          [0,0,0,0,0,0],
          [0,0,0,0,0,0],
          [0,0,0,0,0,0],
          [0,0,0,0,0,0],
          [0,0,0,0,0,0],
        ]
inputArray = np.asarray([[np.transpose(input)]]);
net.blobs['data'].data[...] = inputArray
output = net.forward()
import timeit
print("NeuralNetworkRunTime"+str(timeit.Timer(timehelper).timeit(number=1000)) + "ms")
print np.ravel(output['XOutput'])[0], np.ravel(output['YOutput'])[0]
