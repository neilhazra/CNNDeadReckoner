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
netX = caffe.Net('deployModels.prototxt','Xmodels/solverXAxis_iter_1000.caffemodel',caffe.TEST)
netY = caffe.Net('deployModels.prototxt','Ymodels/solverYAxis_iter_1000.caffemodel',caffe.TEST)
f = h5py.File('HDF5FormattedTrainingData/test_data.h5', 'r')
size = len(f['dataX'])
print(len(f['dataX'])-1)
for i in range(0,len(f['dataX'])-1):
	netX.blobs['data'].data[...] = f['dataX'][i]
	netY.blobs['data'].data[...] = f['dataY'][i]
	outputX = netX.forward()
	outputY = netY.forward()
	print(str(i) + "," + str(np.ravel(outputX['X'])[0]) + "," + str(f['labelX'][i]))
	print(str(i) + "," + str(np.ravel(outputY['Y'])[0]) + "," + str(f['labelY'][i]))
for k in range (0,size):
	for i in range(0,90):
		runX = np.transpose(np.array(f['dataX'][k][0]))
		runY = np.transpose(np.array(f['dataY'][k][0]))
		runX = runX[:i]
		runY = runY[:i]
		for j in range(i,90):
			runX = np.append(runX, [[0,0,0,0,0,0]], axis = 0)
			runY = np.append(runY, [[0,0,0,0,0,0]], axis = 0)
		runX = np.array([np.transpose(runX)])
		runY = np.array([np.transpose(runY)])
		netX.blobs['data'].data[...] = runX
		netY.blobs['data'].data[...] = runY
		outputX = netX.forward()
		outputY = netY.forward()
		s = str(i) + "," + str(np.ravel(outputX['X'])[0]) + "," + str(np.ravel(outputY['Y'])[0])+"," + str(100*f['encoderAcclData'][k][i][0])+ ","
		s = s + str(100*f['encoderAcclData'][k][i][1])+ "," + str(100*f['encoderAcclData'][k][i][2])+ "," + str(100*f['encoderAcclData'][k][i][3])+ ","
		print(s)
		#s = s + str(100*f['encoderAcclData'][k][i][1] + "," + str(100*f['encoderAcclData'][k][i][2])+ "," + str(100*f['encoderAcclData'][k][i][3])
