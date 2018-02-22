#!bin/bash
sudo /home/nvidia/jetson_clocks.sh
PYTHONPATH=/home/nvidia/caffe/python
python /home/nvidia/Documents/MakeHDF5FromRaw.py
python /home/nvidia/Documents/TrainModel.py
