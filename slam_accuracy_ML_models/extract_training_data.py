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
import bisect
import pathlib
import warnings
import zipfile

import numpy as np
import pandas as pd
import rosbag

TOPIC_CAM = "/cam0/image_raw"
TOPIC_METRIC = "/collector/metrics"
IDX = 'stamp'
METRIC_HEADERS = [IDX, 'img_delta', 'blur', 'lin_acc.x', 'lin_acc.y', 'lin_acc.z', 'ang_vel.x', 'ang_vel.y',
                  'ang_vel.z']
ERR = 'err'
RPE = 'rpe'
ATE = 'ate'
DRIFT = 'drift'
ZIP_ERR = "error_array.npy"
ZIP_TS = "timestamps.npy"


def _find_closest(_list, _value):
    idx = bisect.bisect_left(_list, _value)
    if idx > 0 and (abs(_value - _list[idx - 1]) < abs(_value - _list[idx]) or idx == len(_list)):
        return _list[idx - 1]
    else:
        return _list[idx]


def extract_metrics(bag_file) -> pd.DataFrame:
    raw_data = []
    for topic, msg, timestamp in rosbag.Bag(bag_file).read_messages(topics=TOPIC_METRIC):
        row = [msg.header.stamp.to_nsec(), msg.img_delta.to_nsec() / 1e6, msg.blur,
               msg.lin_acc.x, msg.lin_acc.y, msg.lin_acc.z, msg.ang_vel.x, msg.ang_vel.y, msg.ang_vel.z]
        raw_data.append(row)
    return pd.DataFrame(raw_data, columns=METRIC_HEADERS).set_index(IDX)


def process_bags_for_metrics():
    for bag in sorted(pathlib.Path('bags').glob('*_ROV.bag')):
        print(f"Processing bag: {bag}")
        metrics_df = extract_metrics(bag)
        csv_file = f"metrics/{str(bag.stem).rsplit('_', maxsplit=2)[0]}_metrics.csv"
        print(f"Saving metrics to {csv_file}")
        metrics_df.to_csv(csv_file)


def extract_errors(zip_file):
    with zipfile.ZipFile(zip_file, 'r') as zf:
        with zf.open(ZIP_ERR, 'r') as err_file, zf.open(ZIP_TS, 'r') as ts_file:
            errors = (np.load(err_file) * 100).astype(np.float)
            timestamps = (np.load(ts_file) * 1e9).astype(np.int)
    return pd.DataFrame({k: v for k, v in zip([IDX, ERR], [timestamps, errors])}).set_index(IDX)


def _match_by_timestamps(metrics_df, result_df):
    values = []
    missed = 0
    for ts in result_df.index:
        closest_metric_ts = _find_closest(metrics_df.index, ts)
        if (diff := abs(ts - closest_metric_ts)) < 1e6:  # 1ms
            values.append((closest_metric_ts, result_df.loc[ts, ERR]))
        else:
            warnings.warn(f"Closest out-of-bound {ts} <-> {closest_metric_ts}, diff: {diff} > 1e-6")
            missed += 1
    print("Matched metric values:", len(values))
    print("Missed values:", missed)
    matched_df = pd.DataFrame(values, columns=[IDX, ERR]).set_index(IDX)
    return matched_df


# def process_results():
#     for result_zip in sorted(pathlib.Path('accuracy').glob('*.zip')):
#         result_df = extract_rpe(result_zip)
#         print(f"Processing result {result_zip}, (len: {len(result_df)})")
#         dataset_num = str(result_zip.stem).split('_')[2]
#         metrics_file = f"metrics/MH_{dataset_num}_metrics.csv"
#         metrics_df = pd.read_csv(metrics_file, index_col=DATA_INDEX)
#         print(f"Read metrics {metrics_file}, (len: {len(metrics_df)})")
#         merged_df = _merge_datasets(metrics_df, result_df)
#         merged_df.dropna(inplace=True)
#         print("Collected data shape:", merged_df.shape)
#         merged_csv = f"training/{str(result_zip.stem).split('_', maxsplit=1)[-1]}.csv"
#         print(f"Saving data to {merged_csv}")
#         merged_df.to_csv(merged_csv)
#         print('#' * 80)


def process_results():
    for bm in range(1, 6):
        metrics_file = f"metrics/MH_0{bm}_metrics.csv"
        metrics_df = pd.read_csv(metrics_file, index_col=IDX)
        print(f"Read metrics {metrics_file}, (len: {len(metrics_df)})")
        for slam in ('lsd', 'orb', 'rovioli'):
            metrics = {}
            rpe_zip = f"accuracy/rpe_mh_0{bm}_{slam}.zip"
            if pathlib.Path(rpe_zip).exists():
                rpe_df = extract_errors(rpe_zip)
                print(f"Processing result {rpe_zip}, (len: {len(rpe_df)})")
                aligned_rpe = _match_by_timestamps(metrics_df, rpe_df)
                metrics[RPE] = aligned_rpe
            ate_zip = f"accuracy/ate_mh_0{bm}_{slam}.zip"
            if pathlib.Path(ate_zip).exists():
                ate_df = extract_errors(ate_zip)
                print(f"Processing result {ate_zip}, (len: {len(ate_df)})")
                aligned_ate = _match_by_timestamps(metrics_df, ate_df)
                metrics[ATE] = aligned_ate
                print("Calculate drifted ATE...")
                drifted_ate = aligned_ate - aligned_ate.shift()
                metrics[DRIFT] = drifted_ate
            if metrics:
                merged_df = metrics_df.copy()
                for m, df in metrics.items():
                    df.rename(columns={ERR: m}, inplace=True)
                    merged_df = pd.concat([merged_df, df], axis=1)
                merged_df.dropna(inplace=True)
                print("Collected data shape:", merged_df.shape)
                merged_csv = f"training/mh_0{bm}_{slam}.csv"
                print(f"Saving data to {merged_csv}")
                merged_df.to_csv(merged_csv)
                print('#' * 80)


def assemble_training_data():
    process_bags_for_metrics()
    process_results()


if __name__ == '__main__':
    # print(extract_metrics(pathlib.Path("bags/MH_01_easy_ROV.bag")))
    # print(extract_rpe("accuracy/rpe_mh_01_rovioli.zip"))
    # process_bags_for_metrics()
    process_results()
    # assemble_training_data()
