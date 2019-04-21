#!/usr/bin/env bash
p_id=$1
object_name=$2

base_dir=${HOME}/deepgrasp_data/${p_id}/${object_name}
base_dir_f=${HOME}/deepgrasp_data/${p_id}/${object_name}_flipped

for f in ${base_dir_f}/cams/*; do
    fn="${f##*/}"
    fn="${fn%.*}"
    extension="${f##*.}"
    cp ${f} ${base_dir}/cams/${fn}_flipped.${extension}
done