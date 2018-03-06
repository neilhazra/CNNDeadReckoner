from __future__ import print_function
import numpy as np
import h5py
import os
filebasepath = "/home/nvidia/Documents/RealDataTraining/RawInputData"
data = []
y1 = []
y2 = []

features = 6

# reshape the data into desired size, eg. Color*Height*Width=3*32*32
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
        table = []
        filei.readline()
        prevIteration  = 1
        for line in filei:
            inputs = line.split(",")
            if len(inputs)>19:
                iteration = int(float(inputs[0]))
                if not iteration == prevIteration:
                    while np.array(table).shape[0] < 70:
                        table.append(np.zeros(features))
                    data.append([np.transpose(np.array(table))])
                    prevIteration = iteration
                    table = []
                table.append([float(inputs[16]), float(inputs[4]), float(inputs[7]),float(inputs[14]),float(inputs[13])*voltage[iteration-1],float(inputs[6])*voltage[iteration-1]])
        while np.array(table).shape[0] < 70:
            table.append(np.zeros(features))
        data.append([np.transpose(table)])

data = np.array(data)
print(data.shape)
print(len(y1))
print(len(y2))
size = data.shape[0]
train_size = int(round(size*0.95)) #test vs train

x_train = data[:train_size]
x_test = data[train_size:]
# there are two labels for each "image"
y1_train = y1[:train_size]
y1_test = y1[train_size:]
y2_train = y2[:train_size]
y2_test = y2[train_size:]

DIR = "/home/nvidia/Documents/RealDataTraining/HDF5FormattedTrainingData"

h5_train = os.path.join(DIR, 'train_data.h5')
h5_test = os.path.join(DIR, 'test_data.h5')

with h5py.File(h5_train, 'w') as f:
    f['data'] = x_train
    f['labelX'] = y1_train
    f['labelY'] = y2_train

with h5py.File(h5_test, 'w') as f:
    f['data'] = x_test
    f['labelX'] = y1_test
    f['labelY'] = y2_test

text_train = os.path.join(DIR, 'train-path.txt')
with open(text_train, 'w') as f:
    print(h5_train, file = f)

text_test = os.path.join(DIR, 'test-path.txt')
with open(text_test, 'w') as f:
     print(h5_test, file = f)
