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

import matplotlib.pyplot as plt
import pandas as pd
from sklearn.metrics import mean_absolute_error, mean_squared_error, mean_absolute_percentage_error, \
    median_absolute_error
from sklearn.preprocessing import MinMaxScaler
from tensorflow.python.keras import Sequential, Input
from tensorflow.python.keras.callbacks import ModelCheckpoint
from tensorflow.python.keras.layers import LSTM, Dense
from tensorflow.python.keras.metrics import RootMeanSquaredError
from tensorflow.python.keras.optimizer_v2.adam import Adam
from tensorflow.python.keras.preprocessing.sequence import TimeseriesGenerator

from extract_training_data import IDX, DRIFT

SCALE_RANGE = (0, 1)
TRAIN_HEADERS = ['img_delta', 'blur', 'lin_acc.x', 'lin_acc.y', 'lin_acc.z', 'ang_vel.x', 'ang_vel.y', 'ang_vel.z']
TARGET = DRIFT
# TARGET = RPE


def load_data_to_pd(file_name: str or pathlib.Path) -> pd.DataFrame:
    if isinstance(file_name, str):
        file_name = pathlib.Path(file_name)
    df = pd.read_csv(file_name.with_suffix('.csv'), index_col=IDX)
    return df


def get_and_preprocess_data(df: pd.DataFrame or str or pathlib.Path) -> pd.DataFrame:
    # If df is a file name then load it from file
    if isinstance(df, (str, pathlib.Path)):
        df = load_data_to_pd(df)
    column_filter = TRAIN_HEADERS + [TARGET]
    df = df[column_filter]
    df.dropna(inplace=True)
    return df


def get_timeseries_dataset(df, train_size=None, timesteps=1, batch_size=1) -> (TimeseriesGenerator,
                                                                               TimeseriesGenerator):
    scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    df = pd.DataFrame(scaler.fit_transform(df), columns=df.columns)
    t_columns = TARGET
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


def get_metrics(measured, predicted) -> dict:
    mae = mean_absolute_error(measured, predicted)
    rmse = math.sqrt(mean_squared_error(measured, predicted))
    mape = mean_absolute_percentage_error(measured, predicted)
    made = median_absolute_error(measured, predicted)
    return dict(MAE=mae, RMSE=rmse, MAPE=mape, MAD=made)


def get_seq_prediction(model, dataset, timesteps, batch_size) -> (pd.DataFrame, pd.DataFrame):
    target = dataset[[TARGET]].copy()
    dataset.drop(TARGET, axis=1, inplace=True)

    scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    df_scaled = pd.DataFrame(scaler.fit_transform(dataset), columns=dataset.columns)
    t_scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    target_scaled = t_scaler.fit_transform(target)
    train_ds = TimeseriesGenerator(df_scaled, target_scaled, length=timesteps, batch_size=batch_size)

    raw_predicted = model.predict(train_ds)
    if len(raw_predicted.shape) > 2:
        raw_predicted = raw_predicted.reshape(raw_predicted.shape[:2])
    predicted = pd.DataFrame(t_scaler.inverse_transform(raw_predicted), columns=target.columns)
    target = target.iloc[timesteps:, :].reset_index(drop=True)
    return target, predicted


def plot_history(history):
    plt.rcParams["figure.figsize"] = (18, 7)
    plt.plot(history.history['loss'], label='training')
    if 'val_loss' in history.history:
        plt.plot(history.history['val_loss'], label='validation')
    plt.title("Learning curves")
    plt.grid(linestyle='dotted', zorder=0)
    plt.legend()
    plt.tight_layout()
    plt.show()


def plot_prediction(measured, predicted, title="train"):
    plt.rcParams["figure.figsize"] = (18, 7)
    metrics = get_metrics(measured, predicted)
    print("Prediction metrics:")
    pprint.pprint(metrics)
    plt.plot(measured, color="tab:blue", label=f"Measured")
    plt.plot(predicted, color="tab:orange", label=f"Predicted")
    plt.margins(x=0)
    plt.grid(linestyle='dotted', zorder=0)
    plt.legend(loc="upper left")
    plt.title(f"{title} -- RMSE: {metrics['RMSE']}")
    plt.tight_layout()
    plt.show()
    plt.close()


########################################################################################################################

TIMESTEPS = 8
BATCH = 64
EPOCH = 300


def get_stacked_lstm_model(input_shape):
    model = Sequential(name="SLAM_Perf")
    model.add(Input(shape=input_shape, name="Input"))
    model.add(LSTM(units=64, name="Encoder", return_sequences=True))
    model.add(LSTM(units=64, name="Decoder"))
    model.add(Dense(units=32, activation='relu', kernel_initializer="he_normal", name="PE_Dense"))
    model.add(Dense(units=1, activation="linear", name="Out_Dense"))
    model.compile(loss='mse', optimizer=Adam(), metrics=[RootMeanSquaredError()])
    return model


def train_network(file_name="training/mh_01_rovioli.csv", test_file="training/mh_02_rovioli.csv"):
    dataset = get_and_preprocess_data(file_name)
    test_dataset = get_and_preprocess_data(test_file)

    train_ds, _ = get_timeseries_dataset(dataset, timesteps=TIMESTEPS, batch_size=BATCH)
    test_ds, _ = get_timeseries_dataset(test_dataset, timesteps=TIMESTEPS, batch_size=BATCH)
    print(f"Train shape:  {train_ds[0][0].shape}")
    print(f"Test shape:  {test_ds[0][0].shape}")
    input_shape = train_ds[0][0].shape[1:]

    model = get_stacked_lstm_model(input_shape=input_shape)
    model.summary(line_length=100)

    with tempfile.NamedTemporaryFile(prefix="stacked_model_checkpoint_", suffix=".h5", dir=".") as best_model:
        mcp = ModelCheckpoint(filepath=best_model.name, monitor='val_loss', save_best_only=True, save_weights_only=True,
                              verbose=1)
        history = model.fit(train_ds, epochs=EPOCH, validation_data=test_ds, shuffle=False, callbacks=[mcp])
        model.load_weights(best_model.name)

    measured, predicted = get_seq_prediction(model, dataset, timesteps=TIMESTEPS, batch_size=BATCH)
    test_measured, test_predicted = get_seq_prediction(model, test_dataset, timesteps=TIMESTEPS, batch_size=BATCH)

    plot_history(history)
    plot_prediction(measured, predicted, title=file_name)
    plot_prediction(test_measured, test_predicted, title=f"{test_file} (test)")

    model.save("stacked_lstm_model_trained.h5")


if __name__ == '__main__':
    train_network(file_name="training/frame1/mh_01_rovioli.csv", test_file="training/frame1/mh_02_rovioli.csv")
