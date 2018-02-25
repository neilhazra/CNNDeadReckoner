import subprocess
import platform
import copy
import numpy as np
import matplotlib.pyplot as plt
from PIL import Image
import caffe
import caffe.draw
import google.protobuf

def print_network(prototxt_filename, caffemodel_filename):
    _net = caffe.proto.caffe_pb2.NetParameter()
    f = open(prototxt_filename)
    google.protobuf.text_format.Merge(f.read(), _net)
    caffe.draw.draw_net_to_file(_net, prototxt_filename + '.png' )

net = caffe.Net('conv.prototxt', caffe.TEST)
caffe.set_device(0)
caffe.set_mode_gpu()
solver = caffe.get_solver('solver.prototxt')
solver.solve()
print_network('conv_split_regressor.prototxt', '/models/model3_train_0422/solver_iter_500.caffemodel')
net = caffe.Net('deploy_conv_split_regressor.prototxt','models/model3_train_0422/solver_iter_500.caffemodel',caffe.TEST)
inputArray = np.asarray([[[ [1,1,1,1,1,1,1,1,1,1],
                            [1,1,1,1,1,1,1,1,1,1],
                            [1,1,1,1,1,1,1,1,1,1],
                            [1,1,1,1,1,1,1,1,1,1],
                            [1,1,1,1,1,1,1,1,1,1],
                            [1,1,1,1,1,1,1,1,1,1],
                        ]]]);
net.blobs['data'].data[...] = inputArray
output = net.forward()
print np.ravel(output['XOutput'])[0]
print np.ravel(output['YOutput'])[0]
