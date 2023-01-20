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
import collections
import math
import pathlib
import pprint
import tempfile

import numpy as np
import pandas as pd
from imblearn.over_sampling import SMOTE
from imblearn.pipeline import Pipeline
from matplotlib import pyplot as plt
from sklearn.metrics import mean_absolute_error, mean_squared_error, mean_absolute_percentage_error, \
    median_absolute_error, accuracy_score, f1_score, fbeta_score
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import MinMaxScaler, LabelEncoder
from tensorflow.python.keras import Input, Model, Sequential
from tensorflow.python.keras.callbacks import ModelCheckpoint
from tensorflow.python.keras.layers import Dense, Dropout
from tensorflow.python.keras.losses import BinaryCrossentropy, Huber, MeanAbsoluteError
from tensorflow.python.keras.metrics import RootMeanSquaredError, BinaryAccuracy
from tensorflow.python.keras.optimizer_v2.adam import Adam
from tensorflow.python.keras.regularizers import l2

from extract_training_data import IDX, RPE, DRIFT

SCALE_RANGE = (0, 1)
FRAME_HEADERS = ['img_delta', 'blur']
IMU_HEADERS = ['lin_acc.x', 'lin_acc.y', 'lin_acc.z', 'ang_vel.x', 'ang_vel.y', 'ang_vel.z']
# TARGET = DRIFT
TARGET = RPE
TARGET_CLASS = "rpe_class"


def load_data_to_pd(file_name: str or pathlib.Path) -> pd.DataFrame:
    if isinstance(file_name, str):
        file_name = pathlib.Path(file_name)
    df = pd.read_csv(file_name.with_suffix('.csv'), index_col=IDX)
    return df


def preprocess_data(df: pd.DataFrame or str or pathlib.Path, use_imu_size=False, only_imu=False) -> pd.DataFrame:
    # If df is a file name then load it from file
    if isinstance(df, (str, pathlib.Path)):
        df = load_data_to_pd(df)
    acc_header = IMU_HEADERS[:3]
    vel_header = IMU_HEADERS[3:]
    df['lin_acc'] = df[acc_header].pow(2).sum(axis=1).pow(0.5)
    df['ang_vel'] = df[vel_header].pow(2).sum(axis=1).pow(0.5)
    imu_headers = ['lin_acc', 'ang_vel'] if use_imu_size else IMU_HEADERS
    column_filter = imu_headers + [TARGET] if only_imu else FRAME_HEADERS + imu_headers + [TARGET]
    df = df[column_filter]
    df[TARGET] = df[TARGET].shift(-1)
    df.dropna(inplace=True)
    return df


def create_dataset(df: pd.DataFrame) -> (pd.DataFrame, pd.Series):
    scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    training_df = pd.DataFrame(scaler.fit_transform(df), columns=df.columns)
    target = training_df.pop(TARGET)
    return training_df, target


def plot_history(history, metric='loss'):
    plt.rcParams["figure.figsize"] = (18, 7)
    plt.plot(history.history[metric], label='training')
    if f'val_{metric}' in history.history:
        plt.plot(history.history[f'val_{metric}'], label='validation')
    plt.title("Learning curves")
    plt.grid(linestyle='dotted', zorder=0)
    plt.legend()
    plt.tight_layout()
    plt.show()


def get_prediction(model: Model, dataset) -> (pd.DataFrame, pd.DataFrame):
    training_data = dataset.copy()
    target_data = training_data[[TARGET]]
    training_data.drop(columns=TARGET, axis=1, inplace=True)

    scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    df_scaled = pd.DataFrame(scaler.fit_transform(training_data), columns=training_data.columns)
    t_scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    t_scaler.fit(target_data)

    raw_predicted = model.predict(df_scaled)
    predicted = pd.DataFrame(t_scaler.inverse_transform(raw_predicted), columns=target_data.columns)
    target_data = target_data.reset_index(drop=True)
    return target_data, predicted


def serialize_minmax_scaling_params(ds, param_file="scaling_params.npz"):
    target = ds[TARGET].copy()
    ds.drop(TARGET, axis=1, inplace=True)
    input_shift = ds.min().to_numpy(dtype=np.float64)
    print(f"input shift: {input_shift}")
    input_range = (ds.max() - ds.min()).to_numpy(dtype=np.float64)
    print(f"input range: {input_range}")
    output_shift = target.min()
    print(f"output shift: {output_shift}")
    output_range = target.max() - target.min()
    print(f"output range: {output_range}")
    np.savez(param_file, in_shift=input_shift, in_range=input_range, out_shift=output_shift, out_range=output_range)
    print(f"Saved to {param_file}")


########################################################################################################################

def create_class_dataset(df: pd.DataFrame) -> (pd.DataFrame, pd.Series):
    scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    training_df = pd.DataFrame(scaler.fit_transform(df), columns=df.columns)
    target_df = training_df.pop(TARGET_CLASS)
    target_scaler = LabelEncoder()
    target = pd.DataFrame(target_scaler.fit_transform(target_df))
    return training_df, target


def preprocess_class_data(df: pd.DataFrame or str or pathlib.Path, use_imu_size=False, only_imu=False,
                          th=None) -> pd.DataFrame:
    # If df is a file name then load it from file
    if isinstance(df, (str, pathlib.Path)):
        df = load_data_to_pd(df)
    acc_header = IMU_HEADERS[:3]
    vel_header = IMU_HEADERS[3:]
    df['lin_acc'] = df[acc_header].pow(2).sum(axis=1).pow(0.5)
    df['ang_vel'] = df[vel_header].pow(2).sum(axis=1).pow(0.5)
    if th is None:
        threshold = 1.5 * df[RPE].std()
        print(f"Calculated threshold: {threshold}")
    else:
        threshold = th
    df['rpe_class'] = df[RPE].apply(lambda x: 0 if x <= threshold else 1).astype('int')
    df['drift_class'] = df[DRIFT].apply(lambda x: 0 if x <= 0.0 else 1).astype('int')
    imu_headers = ['lin_acc', 'ang_vel'] if use_imu_size else IMU_HEADERS
    column_filter = imu_headers + [TARGET_CLASS] if only_imu else FRAME_HEADERS + imu_headers + [TARGET_CLASS]
    df = df[column_filter]
    df.dropna(inplace=True)
    return df


def get_class_prediction(model: Model, dataset, use_prob=False) -> (pd.DataFrame, pd.DataFrame):
    training_data = dataset.copy()
    target_data = training_data[TARGET_CLASS]
    training_data.drop(columns=TARGET_CLASS, axis=1, inplace=True)

    scaler = MinMaxScaler(feature_range=SCALE_RANGE)
    df_scaled = pd.DataFrame(scaler.fit_transform(training_data), columns=training_data.columns)
    # t_scaler = LabelEncoder()
    # t_scaler.fit(target_data)

    raw_predicted = model.predict(df_scaled)
    # predicted = pd.DataFrame(t_scaler.inverse_transform(raw_predicted.round().astype('int')),
    #                          columns=[TARGET_CLASS])
    raw_predicted = raw_predicted if use_prob else raw_predicted.round().astype('int')
    predicted = pd.DataFrame(raw_predicted, columns=[TARGET_CLASS])
    target_data = target_data.reset_index(drop=True)
    return target_data, predicted


def get_metrics(measured: pd.DataFrame, predicted: pd.DataFrame) -> dict:
    mae = mean_absolute_error(measured, predicted)
    rmse = math.sqrt(mean_squared_error(measured, predicted))
    mape = mean_absolute_percentage_error(measured, predicted)
    made = median_absolute_error(measured, predicted)
    return dict(MAE=mae, RMSE=rmse, MAPE=mape, MAD=made)


def get_class_metrics(measured: pd.DataFrame, predicted: pd.DataFrame) -> dict:
    accuracy = accuracy_score(measured, predicted)
    f1 = f1_score(measured, predicted)
    f0_5 = fbeta_score(measured, predicted, beta=0.5)
    return dict(ACC=f"{accuracy:.3f}", F1=f"{f1:.3f}", F0_5=f"{f0_5:.3f}")


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
    plt.title(f"{title} -- RMSE: {metrics['RMSE']:3f}, MAE: {metrics['MAE']:3f}")
    plt.tight_layout()
    plt.show()
    plt.close()
    return metrics


def plot_class_prediction(measured, predicted, title="train", threshold=None):
    plt.rcParams["figure.figsize"] = (18, 7)
    metrics = get_class_metrics(measured.astype('int'), predicted.astype('int'))
    print(f"Prediction accuracy metrics: {metrics}")
    plt.plot(measured, color="tab:blue", label=f"Measured", linewidth=1.5)
    plt.plot(predicted, color="tab:orange", label=f"Predicted", linewidth=1.0, linestyle="--")
    if threshold:
        plt.gca().axhline(y=threshold, color="tab:red", linestyle="--")
    plt.margins(x=0)
    plt.grid(linestyle='dotted', zorder=0)
    plt.legend(loc="upper left")
    plt.title(f"{title} -- {metrics}")
    plt.tight_layout()
    plt.show()
    plt.close()
    return metrics


########################################################################################################################

# For Regression
USE_IMU_SIZE = False
ONLY_IMU = False
BATCH = 128
EPOCH = 200


# For classification
# USE_IMU_SIZE = False
# ONLY_IMU = False
# BATCH = 128
# EPOCH = 300


def get_dnn_model(input_dim):
    model = Sequential(name="SLAM_errors")
    model.add(Input(shape=input_dim, name="Input"))
    model.add(Dense(units=512, activation="relu", kernel_initializer="he_normal", bias_regularizer=l2(0.2)))
    model.add(Dropout(rate=0.3))
    model.add(Dense(units=512, activation="relu", kernel_initializer="he_normal", bias_regularizer=l2(0.2)))
    model.add(Dropout(rate=0.2))
    model.add(Dense(units=1, activation="linear"))
    model.compile(loss=Huber(), optimizer=Adam(learning_rate=1e-4, beta_1=0.99),
                  metrics=[RootMeanSquaredError(), MeanAbsoluteError()])
    return model


def train_one_model(file_name="training/mh_01_rovioli.csv", test="training/mh_02_rovioli.csv", plotting=True):
    # Load, process and split data
    if isinstance(file_name, (list, tuple)):
        train_data = pd.concat([preprocess_data(f, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU) for f in file_name])
    else:
        train_data = preprocess_data(file_name, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU)
    if isinstance(test, (int, tuple)):
        test_data = pd.concat([preprocess_data(f, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU) for f in test])
    elif isinstance(test, (int, float)):
        train_data, test_data = train_test_split(train_data, test_size=test)
    else:
        test_data = preprocess_data(test, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU)
    train_df, train_target = create_dataset(train_data)
    test_df, test_target = create_dataset(test_data)

    # Train the model
    print(f"Input dim: {train_df.shape}")
    input_dim = train_df.shape[-1]

    model = get_dnn_model(input_dim=input_dim)
    model.summary(line_length=150)

    with tempfile.NamedTemporaryFile(prefix="dnn_model_checkpoint_", suffix=".h5", dir=".") as best_model:
        mcp = ModelCheckpoint(filepath=best_model.name, monitor='val_loss',
                              save_best_only=True, save_weights_only=True, verbose=1)
        history = model.fit(x=train_df, y=train_target, validation_data=(test_df, test_target),
                            epochs=EPOCH, batch_size=BATCH, shuffle=True, callbacks=[mcp])
        model.load_weights(best_model.name)

    # Plot the training/validation metrics
    measured, predicted = get_prediction(model, train_data)
    test_measured, test_predicted = get_prediction(model, test_data)
    if plotting:
        plot_history(history)
        train_metrics = plot_prediction(measured, predicted, title=file_name)
        test_metrics = plot_prediction(test_measured, test_predicted, title=f"{test} (test)")
    else:
        train_metrics = get_metrics(measured, predicted)
        test_metrics = get_metrics(test_measured, test_predicted)

    model.save("dnn_model_trained.h5")
    return train_metrics, test_metrics


def test_model(file_name="training/mh_01_rovioli.csv", test_file="training/mh_02_rovioli.csv"):
    ITERATIONS = 10
    results = [train_one_model(file_name, test_file) for _ in range(ITERATIONS)]
    results = pd.DataFrame(results, columns=['train', 'test'])
    print(results.describe())


########################################################################################################################

def get_dnn_class_model(input_dim):
    model = Sequential(name="SLAM_errors")
    model.add(Input(shape=input_dim, name="Input"))
    model.add(Dense(units=512, activation="relu", kernel_initializer="he_normal", bias_regularizer=l2(0.2)))
    model.add(Dropout(rate=0.5))
    model.add(Dense(units=512, activation="relu", kernel_initializer="he_normal", bias_regularizer=l2(0.1)))
    model.add(Dropout(rate=0.5))
    model.add(Dense(units=1, activation="sigmoid"))
    model.compile(loss=BinaryCrossentropy(from_logits=True), optimizer=Adam(learning_rate=1e-4, beta_2=0.99),
                  metrics=[BinaryAccuracy()])
    return model


def train_one_class_model(file_name="training/mh_01_rovioli.csv", test="training/mh_02_rovioli.csv"):
    # Load, process and split data
    if isinstance(file_name, (list, tuple)):
        orig_data = pd.concat([preprocess_data(f, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU, ) for f in file_name])
        th = 1.5 * orig_data[RPE].std()
        print(f"Calculated train threshold: {th}")
        train_data = pd.concat(
            [preprocess_class_data(f, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU, th=th) for f in file_name])
    else:
        train_data = preprocess_class_data(file_name, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU)
    if isinstance(test, (list, tuple)):
        orig_test = pd.concat(
            [preprocess_class_data(f, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU) for f in test])
        th = 1.5 * orig_test[RPE].std()
        print(f"Calculated test threshold: {th}")
        test_data = pd.concat(
            [preprocess_class_data(f, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU, th=th) for f in test])
    elif isinstance(test, (int, float)):
        train_data, test_data = train_test_split(train_data, test_size=test)
    else:
        test_data = preprocess_class_data(test, use_imu_size=USE_IMU_SIZE, only_imu=ONLY_IMU)
    train_df, train_target = create_class_dataset(train_data)
    test_df, test_target = create_class_dataset(test_data)

    main_stat = collections.Counter(train_target[0])
    print(f"Classes: {main_stat}, minority: {main_stat[1] / main_stat[0]:.3f} %")
    print(f"Input dim: {train_df.shape}")
    # train_df, train_target = SMOTE().fit_resample(train_df, train_target)
    # sampler = Pipeline([('over_sample', SMOTE(sampling_strategy=0.5)),
    #                     ('under_sample', RandomUnderSampler(sampling_strategy=0.8))])
    sampler = Pipeline([('over_sample', SMOTE(sampling_strategy=0.65))])
    train_df, train_target = sampler.fit_resample(train_df, train_target)
    print(f"Resampled (SMOTE + RandomUnder) Input dim: {train_df.shape}")
    print(f"Classes: {collections.Counter(train_target[0])}")

    # Train the model
    input_dim = train_df.shape[-1]
    model = get_dnn_class_model(input_dim=input_dim)
    model.summary(line_length=150)

    with tempfile.NamedTemporaryFile(prefix="dnn_class_model_checkpoint_", suffix=".h5", dir=".") as best_model:
        mcp = ModelCheckpoint(filepath=best_model.name, monitor='val_loss',
                              save_best_only=True, save_weights_only=True, verbose=1)
        history = model.fit(x=train_df, y=train_target, validation_data=(test_df, test_target),
                            # epochs=EPOCH, batch_size=BATCH, shuffle=True, callbacks=[mcp])
                            epochs=EPOCH, batch_size=BATCH, shuffle=True)
        # model.load_weights(best_model.name)

    # Plot the training/validation metrics
    measured, predicted = get_class_prediction(model, train_data, use_prob=False)
    test_measured, test_predicted = get_class_prediction(model, test_data, use_prob=False)
    # plot_history(history, metric="binary_accuracy")
    plot_history(history)
    train_metrics = plot_class_prediction(measured, predicted, title=file_name, threshold=0.5)
    test_metrics = plot_class_prediction(test_measured, test_predicted, title=f"{test} (test)", threshold=0.5)

    model.save("dnn_class_model_trained.h5")
    return train_metrics, test_metrics


if __name__ == '__main__':
    # train_one_model(file_name="training/mh_01_rovioli.csv", test_file="training/mh_02_rovioli.csv")
    # train_one_model(file_name="training/mh_02_rovioli.csv", test_file="training/mh_01_rovioli.csv")
    # train_one_model(file_name="training/mh_04_rovioli.csv", test_file="training/mh_01_rovioli.csv")
    # train_one_model(file_name="training/mh_01_rovioli.csv", test_file="training/mh_04_rovioli.csv")

    # train_one_model(file_name=["training/mh_03_rovioli.csv", "training/mh_04_rovioli.csv"],
    #                 test_file="training/mh_01_rovioli.csv")
    # train_one_model(file_name=["training/frame10/mh_01_rovioli.csv", "training/frame10/mh_04_rovioli.csv"],
    #                 test_file="training/frame10/mh_03_rovioli.csv")

    # train_one_model(
    #     file_name=["training/mh_02_rovioli.csv", "training/mh_03_rovioli.csv", "training/mh_04_rovioli.csv"],
    #     test_file="training/mh_01_rovioli.csv")

    # test_model(file_name=["training/mh_01_rovioli.csv", "training/mh_04_rovioli.csv"],
    #            test_file="training/mh_03_rovioli.csv")

    # train_one_class_model(file_name=["training/frame10/mh_02_rovioli.csv", "training/frame10/mh_03_rovioli.csv",
    #                                  "training/frame10/mh_04_rovioli.csv"],
    #                       test_file="training/frame10/mh_01_rovioli.csv")

    # train_one_model(file_name=["training/frame10/mh_01_rovioli.csv", "training/frame10/mh_02_rovioli.csv",
    #                            "training/frame10/mh_03_rovioli.csv", "training/frame10/mh_04_rovioli.csv"],
    #                 test=0.3)
    train_one_model(file_name=["training/frame10/mh_01_lsd.csv", "training/frame10/mh_02_lsd.csv",
                               "training/frame10/mh_03_lsd.csv", "training/frame10/mh_04_lsd.csv"],
                    test=0.3)
    # train_one_class_model(file_name=["training/frame10/mh_01_rovioli.csv", "training/frame10/mh_02_rovioli.csv",
    #                                  "training/frame10/mh_03_rovioli.csv", "training/frame10/mh_04_rovioli.csv"],
    #                       test=0.3)
