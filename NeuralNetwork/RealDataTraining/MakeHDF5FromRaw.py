from __future__ import print_function
import numpy as np
import h5py
import os
filebasepath = "/home/nvidia/Documents/RealDataTraining/RawInputData"
dataX = []
dataY = []
y1 = []
y2 = []

encoderAcclData = [] #Ex, Ey, Ax, Ay 4x90x1000

features = 6

# reshape the dataX into desired size, eg. Color*Height*Width=3*32*32
files = sorted(set([os.path.splitext(_file)[0] for _file in os.listdir(filebasepath)]))
print(files)
for filename in sorted(files):
    voltage = []
    with open(os.path.join(filebasepath, filename+'.txt')) as filei:
        filei.readline()
        for line in filei:
            inputs = line.split(",")
            if len(inputs)>13:
                y1.append(float(inputs[3])) #X
                y2.append(float(inputs[4])) #Y
                voltage.append(float(inputs[2]))
    with open(os.path.join(filebasepath, filename+'.csv')) as filei:
        tableX = []
        tableY = []
        encoderAcclTable = []
        filei.readline()
        prevIteration  = 1
        print("Reading File " + filename)
        for line in filei:
            inputs = line.split(",")
            if len(inputs)>19:
                iteration = int(float(inputs[0]))
                if not iteration == prevIteration:
                    while np.array(tableX).shape[0] < 90:
                        tableX.append(np.zeros(features))
                        tableY.append(np.zeros(features))
                        encoderAcclTable.append(np.zeros(4))
                    dataX.append([np.transpose(np.array(tableX))])
                    dataY.append([np.transpose(np.array(tableY))])
                    encoderAcclData.append(np.array(encoderAcclTable))
                    prevIteration = iteration
                    tableX = []
                    tableY = []
                    encoderAcclTable = []
                tableX.append([float(inputs[8]), float(inputs[9]), float(inputs[14]),float(inputs[7]),float(inputs[6])*voltage[iteration-1],float(inputs[13])*voltage[iteration-1]])
                #              dAcclX,           dEncX,             dR,              dL,             Pr*V                                       , Pl*v
                tableY.append([float(inputs[15]), float(inputs[16]), float(inputs[14]),float(inputs[7]),float(inputs[6])*voltage[iteration-1],float(inputs[13])*voltage[iteration-1]])
                #              dAcclY,           dEncY,             dR,              dL,             Pr*V                                       , Pl*v

                encoderAcclTable.append([float(inputs[5]), float(inputs[12]), float(inputs[4]),float(inputs[11])])
                #                             Ex,           Ey,                Acclx,           Accly
        while np.array(tableX).shape[0] < 90:
            tableX.append(np.zeros(features))
            tableY.append(np.zeros(features))
            encoderAcclTable.append(np.zeros(4))
        dataX.append([np.transpose(tableX)])
        dataY.append([np.transpose(tableY)])
        encoderAcclData.append(np.array(encoderAcclTable))

dataX = np.array(dataX)
dataY = np.array(dataY)
encoderAcclData = np.array(encoderAcclData)
print("Created ArrayX " + str(dataX.shape))
print("Created ArrayY " + str(dataY.shape))
print("Created ArrayEncAccl " + str(encoderAcclData.shape))

size = dataX.shape[0]
train_size = int(round(size*0.95)) #test vs train

x_trainX = dataX[:train_size]
x_testX= dataX[train_size:]
x_trainY = dataY[:train_size]
x_testY= dataY[train_size:]

# there are two labels for each "image"
y1_train = y1[:train_size]
y1_test = y1[train_size:]
y2_train = y2[:train_size]
y2_test = y2[train_size:]

DIR = "/home/nvidia/Documents/RealDataTraining/HDF5FormattedTrainingData"

h5_train = os.path.join(DIR, 'train_data.h5')
h5_test = os.path.join(DIR, 'test_data.h5')

with h5py.File(h5_train, 'w') as f:
    f['dataX'] = x_trainX
    f['dataY'] = x_trainY
    f['encoderAcclData']  = encoderAcclData
    f['labelX'] = y1_train
    f['labelY'] = y2_train

with h5py.File(h5_test, 'w') as f:
    f['dataX'] = x_testX
    f['dataY'] = x_testY
    f['encoderAcclData']  = encoderAcclData
    f['labelX'] = y1_test
    f['labelY'] = y2_test

text_train = os.path.join(DIR, 'train-path.txt')
with open(text_train, 'w') as f:
    print(h5_train, file = f)

text_test = os.path.join(DIR, 'test-path.txt')
with open(text_test, 'w') as f:
     print(h5_test, file = f)
