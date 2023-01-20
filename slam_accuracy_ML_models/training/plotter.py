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
import pathlib

import pandas as pd
from matplotlib import pyplot as plt


def plot_accuracy_compare(csv_file):
    result = pd.read_csv(csv_file, index_col='stamp')
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(20, 10))
    axes[0].plot(result['ate'], label="ATE")
    axes[0].set_ylabel("Error [cm]")
    axes[0].margins(x=0)
    axes[0].grid(linestyle=':')
    axes[0].legend()

    axes[1].plot(result['rpe'], label="RPE [1 frame]")
    axes[1].plot(result['drift'], label="ATE drift")
    axes[1].set_xlabel("Timestamps")
    axes[1].set_ylabel("Error [cm]")
    axes[1].margins(x=0)
    axes[1].grid(linestyle=':')
    axes[1].legend()
    plt.suptitle(csv_file)
    plt.tight_layout()
    plt.show()
    plt.close()


def plot_all_results(csv_filter="*.csv"):
    for csv in sorted(pathlib.Path('.').glob(csv_filter)):
        plot_accuracy_compare(csv)


if __name__ == '__main__':
    # plot_accuracy_compare("mh_01_rovioli.csv")
    # plot_all_results("frame1/*_rovioli.csv")
    plot_all_results("*_rovioli.csv")
    # plot_all_results()
