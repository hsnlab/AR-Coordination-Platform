#!/usr/bin/env bash

trap exit INT

for i in *; do
  if [ -d "$i" ]; then
    evo_traj euroc -v ${i}/data.csv -v --full_check --save_as_tum
    mv -v data.tum euroc_${i}_gt.tum
  fi
done
