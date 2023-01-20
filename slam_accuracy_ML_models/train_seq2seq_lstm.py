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
import math
import pathlib
import pprint
import tempfile

import pandas as pd
from matplotlib import pyplot as plt
from sklearn.metrics import (mean_absolute_percentage_error, mean_absolute_error, mean_squared_error,
                             median_absolute_error)
from sklearn.preprocessing import MinMaxScaler
from tensorflow.python.keras import Input, Model
from tensorflow.python.keras.callbacks import ModelCheckpoint
from tensorflow.python.keras.layers import Bidirectional, LSTM, RepeatVector, TimeDistributed, Dense
from tensorflow.python.keras.metrics import RootMeanSquaredError
from tensorflow.python.keras.optimizer_v2.adam import Adam
from tensorflow.python.keras.preprocessing.sequence import TimeseriesGenerator
from tensorflow.python.keras.regularizers import l2

from extract_training_data import IDX, DRIFT, RPE, ATE

IMU_HEADERS = ['lin_acc.x', 'lin_acc.y', 'lin_acc.z', 'ang_vel.x', 'ang_vel.y', 'ang_vel.z']
DRIFT_Q50 = 'drift_50'
DRIFT_Q70 = 'drift_70'
DRIFT_Q90 = 'drift_90'
SCALE_RANGE = (0, 1)


def load_data_to_pd(file_name: str or pathlib.Path) -> pd.DataFrame:
    if isinstance(file_name, str):
        file_name = pathlib.Path(file_name)
    df = pd.read_csv(file_name.with_suffix('.csv'), index_col=IDX)
    return df


def get_and_preprocess_data(file_name, q_window=10) -> pd.DataFrame:
    df = load_data_to_pd(file_name)
    # Add plus 1 to window to have the current exec_time and the next q_window element as the base for percentile calc.
    df[DRIFT_Q50] = df[DRIFT].rolling(q_window + 1).quantile(0.5).shift(-1 * q_window)
    df[DRIFT_Q70] = df[DRIFT].rolling(q_window + 1).quantile(0.7).shift(-1 * q_window)
    df[DRIFT_Q90] = df[DRIFT].rolling(q_window + 1).quantile(0.9).shift(-1 * q_window)
    df.drop(columns=[RPE, ATE], inplace=True)
    df.dropna(inplace=True)
    return df


def get_timeseries_dataset(df, train_size=None, timesteps=1, batch_size=1) -> (TimeseriesGenerator,
                                                                               TimeseriesGenerator):
    scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    df = pd.DataFrame(scaler.fit_transform(df), columns=df.columns)
    t_columns = [DRIFT_Q50, DRIFT_Q70, DRIFT_Q90]
    # target = df[t_columns].copy().values.tolist()
    # Predict the accuracy of the latest t. frame, not the following t+1. frame
    target = df[t_columns].shift(1).values.tolist()
    df.drop(t_columns, axis=1, inplace=True)
    if train_size is None:
        train_ds = test_ds = TimeseriesGenerator(df, target, length=timesteps, batch_size=batch_size)
    else:
        train_ds = TimeseriesGenerator(df, target, length=timesteps, batch_size=batch_size, end_index=train_size)
        test_ds = TimeseriesGenerator(df, target, length=timesteps, batch_size=batch_size, start_index=train_size)
    return train_ds, test_ds


def get_seq_prediction(model, dataset, timesteps, batch_size) -> (pd.DataFrame, pd.DataFrame):
    t_columns = [DRIFT_Q50, DRIFT_Q70, DRIFT_Q90]
    target = dataset[t_columns].copy()
    dataset.drop(t_columns, axis=1, inplace=True)

    scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    df_scaled = pd.DataFrame(scaler.fit_transform(dataset), columns=dataset.columns)
    t_scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    target_scaled = t_scaler.fit_transform(target)
    train_ds = TimeseriesGenerator(df_scaled, target_scaled, length=timesteps, batch_size=batch_size)

    raw_predicted = model.predict(train_ds)
    if len(raw_predicted.shape) > 2:
        raw_predicted = raw_predicted.reshape(raw_predicted.shape[:2])
    predicted = pd.DataFrame(t_scaler.inverse_transform(raw_predicted), columns=target.columns)
    target = target.loc[timesteps:, :].reset_index(drop=True)
    return target, predicted


def get_metrics(measured, predicted) -> dict:
    idx = predicted.index[predicted.notnull()]
    mae = mean_absolute_error(measured[idx], predicted[idx])
    rmse = math.sqrt(mean_squared_error(measured[idx], predicted[idx]))
    mape = mean_absolute_percentage_error(measured[idx], predicted[idx])
    made = median_absolute_error(measured[idx], predicted[idx])
    return dict(MAE=mae, RMSE=rmse, MAPE=mape, MAD=made)


def get_seq_metrics(measured, predicted) -> (list, dict):
    metrics = [get_metrics(measured.iloc[:, i], predicted.iloc[:, i])
               for i in range(predicted.shape[1])]
    sum_metrics = get_metrics(measured=pd.Series(measured.values.flatten(order='F')),
                              predicted=pd.Series(predicted.values.flatten(order='F')))
    return sum_metrics, metrics


def plot_history(history):
    plt.rcParams["figure.figsize"] = (15, 7)
    plt.plot(history.history['loss'], label='training')
    plt.plot(history.history['val_loss'], label='validation')
    plt.title("Learning curves")
    # plt.ylim(top=0.02)
    plt.grid(linestyle='dotted', zorder=0)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_seq_prediction(measured, predicted, title="train"):
    sum_metrics, metrics = get_seq_metrics(measured, predicted)
    print("Sum metrics:")
    pprint.pprint(sum_metrics)
    print("Q1, Q2, Q3 metrics:")
    pprint.pprint(metrics)
    metrics_rmse = [m.get('RMSE') for m in metrics]
    fig, _ = plt.subplots(nrows=len(measured.columns), ncols=1, sharex=True, sharey=True, figsize=(20, 10))
    for i, c in enumerate(measured):
        fig.axes[i].plot(measured[c], color="tab:blue", label=f"Measured_{c}")
        fig.axes[i].plot(predicted[c], color="tab:orange", label=f"Predicted_{c}")
        fig.axes[i].margins(x=0)
        fig.axes[i].grid(linestyle='dotted', zorder=0)
        fig.axes[i].legend(loc="upper left")
    fig.axes[0].set_title(f"{title} -- RMSE (q50, q70, q90): {metrics_rmse} ms - Sum RMSE: {sum_metrics['RMSE']} ms")
    plt.tight_layout()
    plt.show()
    plt.close()


########################################################################################################################

class CombinedModelCheckpoint(ModelCheckpoint):

    def __init__(self, filepath, combined=True, delay=0, verbose=1, **kwargs):
        self.epoch_delay = delay
        self.combined = combined
        monitor = 'combined' if combined else 'val_loss'
        super(CombinedModelCheckpoint, self).__init__(filepath, monitor=monitor, verbose=verbose, save_best_only=True,
                                                      save_weights_only=True, mode='auto', save_freq='epoch', **kwargs)

    def on_epoch_end(self, epoch, logs=None):
        self.epochs_since_last_save += 1
        # pylint: disable=protected-access
        if self.combined:
            logs['combined'] = logs['loss'] + logs['val_loss']
        if self.save_freq == 'epoch':
            if epoch == 1 or epoch >= self.epoch_delay:
                self._save_model(epoch=epoch, logs=logs)


def get_encoder_decoder_lstm2_model(input_shape):
    input_layer = Input(shape=input_shape, name="Input")
    encoder = Bidirectional(LSTM(units=64, dropout=0.25), name="Encoder")(input_layer)
    context_vector = RepeatVector(n=3, name="Context_Vector")(encoder)
    decoder = LSTM(units=64, return_sequences=True, dropout=0.25, name="Decoder")(context_vector)
    extractor = TimeDistributed(Dense(units=32, activation='relu', kernel_initializer="he_normal",
                                      bias_regularizer=l2(5e-5)), name="Percentile_Extractor")(decoder)
    output_layer = TimeDistributed(Dense(units=1, activation="linear"), name="Output")(extractor)
    model = Model(inputs=input_layer, outputs=output_layer)
    model.compile(loss='MSE', optimizer=Adam(learning_rate=5e-4, beta_1=0.99, clipnorm=1),
                  metrics=[RootMeanSquaredError()])
    return model


WINDOW = 10
TIMESTEPS = 8
BATCH = 64
EPOCH = 300


def train_network(file_name="training/mh_04_rovioli.csv", test_file="training/mh_01_rovioli.csv"):
    """Train network"""
    dataset = get_and_preprocess_data(file_name, q_window=WINDOW)
    test_dataset = get_and_preprocess_data(test_file, q_window=WINDOW)
    train_ds, _ = get_timeseries_dataset(dataset, timesteps=TIMESTEPS, batch_size=BATCH)
    test_ds, _ = get_timeseries_dataset(test_dataset, timesteps=TIMESTEPS, batch_size=BATCH)
    print(f"Train shape:  {train_ds[0][0].shape}")
    print(f"Test shape:  {test_ds[0][0].shape}")
    input_shape = train_ds[0][0].shape[1:]

    model = get_encoder_decoder_lstm2_model(input_shape=input_shape)
    model.summary(line_length=150)

    with tempfile.NamedTemporaryFile(prefix="seq2seq_model_checkpoint_", suffix=".h5", dir=".") as best_model:
        mcp = CombinedModelCheckpoint(filepath=best_model.name, combined=False, delay=400)
        # history = model.fit(train_ds, epochs=EPOCH, validation_data=test_ds, shuffle=False, callbacks=[mcp])
        history = model.fit(train_ds, epochs=EPOCH, validation_data=test_ds, shuffle=False)
        # model.load_weights(best_model.name)

    measured, predicted = get_seq_prediction(model, dataset, timesteps=TIMESTEPS, batch_size=BATCH)
    test_measured, test_predicted = get_seq_prediction(model, test_dataset, timesteps=TIMESTEPS, batch_size=BATCH)

    plot_history(history)
    plot_seq_prediction(measured=measured, predicted=predicted)
    plot_seq_prediction(measured=test_measured, predicted=test_predicted, title="test")

    model.save("lstm2_model_trained.h5")


if __name__ == '__main__':
    train_network()
