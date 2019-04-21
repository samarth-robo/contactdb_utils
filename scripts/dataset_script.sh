#!/usr/bin/env bash
set -x

INSTR=handoff
for id in $(seq 1 24); do
  echo "Processing ${id}_${INSTR}"
  python generate_object_masks.py --data_dir ~/deepgrasp_data/data/ --sess full${id}_${INSTR}
done
for id in $(seq 30 33); do
  echo "Processing ${id}_${INSTR}"
  python generate_object_masks.py --data_dir ~/deepgrasp_data/data2/ --sess full${id}_${INSTR}
done
for id in $(seq 25 29; seq 34 50); do
  echo "Processing ${id}_${INSTR}"
  python generate_object_masks.py --data_dir ~/deepgrasp_data/data3/ --sess full${id}_${INSTR}
done

INSTR=use
for id in $(seq 1 28); do
  echo "Processing ${id}_${INSTR}"
  python generate_object_masks.py --data_dir ~/deepgrasp_data/data/ --sess full${id}_${INSTR}
done
for id in $(seq 30 33); do
  echo "Processing ${id}_${INSTR}"
  python generate_object_masks.py --data_dir ~/deepgrasp_data/data2/ --sess full${id}_${INSTR}
done
for id in $(seq 29 29; seq 34 50); do
  echo "Processing ${id}_${INSTR}"
  python generate_object_masks.py --data_dir ~/deepgrasp_data/data3/ --sess full${id}_${INSTR}
done
set +x
