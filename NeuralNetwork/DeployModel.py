import subprocess
import platform
import copy
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import caffe
import caffe.draw
import google.protobuf
import h5py
caffe.set_device(0)
caffe.set_mode_gpu()
def timehelper():
	inputArray = np.asarray([[np.transpose(input)]]);
	net.blobs['data'].data[...] = inputArray
	output = net.forward()
net = caffe.Net('deploy.prototxt','models/solver_iter_5000.caffemodel',caffe.TEST)

f = h5py.File('HDF5FormattedTrainingData/test_data.h5', 'r')
print(len(f['data'])-1)
for i in range(0,len(f['data'])-1):
	print(i)
	net.blobs['data'].data[...] = f['data'][i]
	output = net.forward()
	print("NeuralNetworkX " + str(np.ravel(output['XOutput'])[0]) + " ActualX " + str(f['labelX'][i]))
	print("NeuralNetworkY " + str(np.ravel(output['YOutput'])[0]) + " ActualY " + str(f['labelY'][i]))

#import timeit
#print("NeuralNetworkRunTime "+str(timeit.Timer(timehelper).timeit(number=1000)) + "ms")
