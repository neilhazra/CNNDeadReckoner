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
    '''
    Draw the ANN architecture
    '''
    _net = caffe.proto.caffe_pb2.NetParameter()
    f = open(prototxt_filename)
    google.protobuf.text_format.Merge(f.read(), _net)
    caffe.draw.draw_net_to_file(_net, prototxt_filename + '.png' )
    print('Draw ANN done!')
def print_network_weights(prototxt_filename, caffemodel_filename):
    '''
    For each ANN layer, print weight heatmap and weight histogram
    '''
    net = caffe.Net(prototxt_filename,caffemodel_filename, caffe.TEST)
    for layer_name in net.params:
        # weights heatmap
        arr = net.params[layer_name][0].data
        plt.clf()
        fig = plt.figure(figsize=(10,10))
        ax = fig.add_subplot(111)
        cax = ax.matshow(arr, interpolation='none')
        fig.colorbar(cax, orientation="horizontal")
        plt.savefig('{0}_weights_{1}.png'.format(caffemodel_filename, layer_name), dpi=100, format='png', bbox_inches='tight') # use format='svg' or 'pdf' for vectorial pictures
        plt.close()
        # weights histogram
        plt.clf()
        plt.hist(arr.tolist(), bins=20)
        plt.savefig('{0}_weights_hist_{1}.png'.format(caffemodel_filename, layer_name), dpi=100, format='png', bbox_inches='tight') # use format='svg' or 'pdf' for vectorial pictures
        plt.close()
net = caffe.Net('conv.prototxt', caffe.TEST)
caffe.set_device(0)
caffe.set_mode_gpu()
solver = caffe.get_solver('solver.prototxt')
solver.solve()
print_network('conv.prototxt', '/models/model3_train_0422/solver_iter_500.caffemodel')
net = caffe.Net('deploy.prototxt','models/model3_train_0422/solver_iter_500.caffemodel',caffe.TEST)
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
