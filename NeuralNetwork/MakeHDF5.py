from __future__ import print_function
import numpy as np
import h5py
import os
data = []
y1 = []
y2 = []
# number of observations or number of images
size = 6
# reshape the data into desired size, eg. Color*Height*Width=3*32*32
with open("/home/nvidia/Documents/RawTrainingData/ResultData.txt") as resultData:
    for line in resultData:
        inputs = line.split(",")
        y1.append(float(inputs[0]))
        y2.append(float(inputs[1]))

for filename in os.listdir("/home/nvidia/Documents/RawTrainingData/RunData"):
    with open(os.path.join("/home/nvidia/Documents/RawTrainingData/RunData", filename)) as filei:
        table = []
        for line in filei:
            inputs = line.split(",")
            table.append([float(i) for i in inputs])
        data.append(table)

data = np.reshape(data, (6, 1, 6,  10))

# shuffle the dataset, make 4/5 be training and 1/5 be testing
train_size = int(round(size*0.8))

x_train = data[:train_size]
x_test = data[train_size:]

# there are two labels for each "image"
y1_train = y1[:train_size]
y1_test = y1[train_size:]
y2_train = y2[:train_size]
y2_test = y2[train_size:]

DIR = "/home/nvidia/Documents/NNInputData"

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
