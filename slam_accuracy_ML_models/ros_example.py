# Copyright 2021 Janos Czentye
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os

import numpy as np
from tensorflow.compat.v1.keras.backend import get_session, set_session
from tensorflow.compat.v1.keras.models import model_from_json


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


if __name__ == '__main__':
    # pip3 install gast==0.2.2 'h5py<3.0.0' 'numpy<1.17' tensorflow==1.15.5
    mw = ModelWrapper(model_cfg="rov_dnn_model_cfg.json", model_weights="rov_dnn_model_weights.h5",
                      scaling_params="rov_dnn_scaling_params.npz")

    ###   img_delta = img_frame[t].stamp - img_frame[t-1].stamp
    ###   blur ==>
    ###      converter = CvBridge()
    ###      blur = Laplacian(converter.imgmsg_to_cv2(img_msg, desired_encoding="passthrough"),
    ###                       ddepth=CV_16S, ksize=1).var()

    ###           img_delta,  blur,  lin_acc.x, lin_acc.y, lin_acc.z, ang_vel.x, ang_vel.y, ang_vel.z
    test_inputs = [[42.69886, 1106.67111, 9.84751, 0.64560, -2.51704, 0.01117, 0.058643, 0.06632],
                   [38.77854, 382.66816, 9.29997, 0.52302, -4.88698, -0.04607, 0.28693, 0.05235],
                   [50.56548, 1086.52080, 7.33047, -0.52302, -2.62327, -0.05864, 0.05096, -0.06283]]
    test_output = [0.75536, 3.18182, 4.43370]
    ###
    predictions = [mw.predict(data) for data in test_inputs]
    print(f"Outputs:       {test_output}")
    print(f"Predictions:   {predictions}")
