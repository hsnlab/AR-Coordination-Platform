#!/usr/bin/env python3

import sys
print("python version")
print(sys.version)
import time

import os
import numpy as np
from cv2 import Laplacian, CV_16S
import rospy
from cv_bridge import CvBridge
# from cv_bridge.boost.cv_bridge_boost import getCvType

# import tensorflow
from tensorflow.compat.v1.keras.backend import get_session, set_session
from tensorflow.compat.v1.keras.models import model_from_json
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String



class ModelWrapper(object):
    """Wrapping LSTM model creation and invocation"""
    MODEL_DIR = "models"
    DEF_MODEL_CFG = "model_cfg.json"
    DEF_MODEL_WEIGHTS = "model_weights.h5"
    DEF_SCALING_PARAMS = "scaling_params.npz"
    MODEL_INPUT_SHAPE = (1, 8)
    TARGET_SHAPE = (1,)

    def __init__(self, model_cfg=DEF_MODEL_CFG, model_weights=DEF_MODEL_WEIGHTS, scaling_params=DEF_SCALING_PARAMS):
        _base_dir = os.path.dirname(__file__)
        self._session = get_session()
        with open(os.path.join(_base_dir, self.MODEL_DIR, model_cfg)) as cfg:
            self.model = model_from_json(cfg.read())
        # rospy.loginfo("Model[%s] is generated from:  %s", self.model.name, model_cfg)
        print("Model[%s] is generated from:  %s" % (self.model.name, model_cfg))
        self.model.load_weights(os.path.join(_base_dir, self.MODEL_DIR, model_weights))
        # rospy.loginfo("Model weights are loaded from:  %s", model_weights)
        print("Model weights are loaded from:  %s" % model_weights)
        params = np.load(os.path.join(_base_dir, self.MODEL_DIR, scaling_params), allow_pickle=False)
        self._in_shift, self._in_range = params['in_shift'], params['in_range']
        self._out_shift, self._out_range = params['out_shift'], params['out_range']
        # rospy.loginfo("Scaling parameters are loaded from:  %s", scaling_params)
        print("Scaling parameters are loaded from:  %s" % scaling_params)

    def _preprocess_features(self, data):
        scaled = (data - self._in_shift) / self._in_range
        return scaled.reshape(self.MODEL_INPUT_SHAPE)

    def _postprocess_target(self, predicted):
        scaled = predicted.reshape(self.TARGET_SHAPE) * self._out_range + self._out_shift
        # return scaled.flatten()
        return scaled[0]

    def predict(self, data):
        model_input = self._preprocess_features(data)
        with self._session.graph.as_default():
            set_session(self._session)
            y_hat = self.model.predict(model_input)
        return self._postprocess_target(y_hat)





class Callbacks(object):

    def __init__(self):
        self.last_img_stamp = None
        # self.predict_list = 0
        self.last_imu_msg = None
        self.pub = rospy.Publisher('rov_prediction',String,queue_size=10)
        self.listener()



    def imu_callback(self, data):
        self.last_imu_msg = data


    def callback(self, data):
        # asd = last_img_stamp
        if self.last_img_stamp is None:
            self.last_img_stamp = data.header.stamp
        else:
            img_delta = data.header.stamp - self.last_img_stamp
            img_delta = float(img_delta.to_sec()*1000)
            self.last_img_stamp = data.header.stamp

        # blur
        converter = CvBridge()
        # blur = Laplacian(np.frombuffer(data, dtype=np.uint8).reshape("720","1280", -1))
        blur = Laplacian(converter.imgmsg_to_cv2(data, desired_encoding="passthrough"),
                         ddepth=CV_16S, ksize=1).var()

        input_data = [img_delta, blur, self.last_imu_msg.linear_acceleration.x, self.last_imu_msg.linear_acceleration.y,
                 self.last_imu_msg.linear_acceleration.z, self.last_imu_msg.angular_velocity.x, self.last_imu_msg.angular_velocity.y,
                 self.last_imu_msg.angular_velocity.z]

        # print("\ninput_data: ", input_data)
        # _start = time.time()
        predictions = mw.predict(input_data) * 25.7838718752368


        msg = str(data.header.stamp.secs) + "." + str(data.header.stamp.nsecs) + ";" + str(predictions)
        # print(msg)
        self.pub.publish(msg)

        # result = time.time()-_start

        # print("\nresult: ", result)

        ###           img_delta,  blur,  lin_acc.x, lin_acc.y, lin_acc.z, ang_vel.x, ang_vel.y, ang_vel.z
        # test_inputs = [[42.69886, 1106.67111, 9.84751, 0.64560, -2.51704, 0.01117, 0.058643, 0.06632],
        #                [38.77854, 382.66816, 9.29997, 0.52302, -4.88698, -0.04607, 0.28693, 0.05235],
        #                [50.56548, 1086.52080, 7.33047, -0.52302, -2.62327, -0.05864, 0.05096, -0.06283]]
        # test_output = [0.75536, 3.18182, 4.43370]
        ###
        # predictions = [mw.predict(data) for data in test_inputs]
        # print(f"Outputs:       {test_output}")
        # print("Predictions: ", str(predictions)) #, "result: ", result, "input_data: ", input_data)



    def listener(self):
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.

        print("listener2")
        rospy.init_node('listener2', anonymous=True)

        rospy.Subscriber("cam0/image_raw", Image, self.callback)
        rospy.Subscriber("imu0", Imu, self.imu_callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    # mw = ModelWrapper(model_cfg="lsd_dnn_model_cfg.json", model_weights="lsd_dnn_model_weights.h5",
    #                   scaling_params="lsd_dnn_scaling_params.npz")

    mw = ModelWrapper(model_cfg="rov_dnn_model_cfg.json", model_weights="rov_dnn_model_weights.h5",
                      scaling_params="rov_dnn_scaling_params.npz")
    c = Callbacks()

