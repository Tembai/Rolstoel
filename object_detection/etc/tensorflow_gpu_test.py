import tensorflow as tf 

if tf.test.gpu_device_name(): 

    print('Default GPU Device:{}'.format(tf.test.gpu_device_name()))

else:

    print("Please install GPU version of TF")

#from tensorflow.python.client import device_lib 
#print(device_lib.list_local_devices())
