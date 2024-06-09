from __future__ import print_function, division
import sys
import csv
import argparse
import os
import io

from time import time

from src.bvhplayer_skeleton import process_bvhfile, process_bvhkeyframe

def open_csv(filename, mode='r'):
    """Open a csv file in proper mode depending on Python version."""
    if sys.version_info < (3,):
        return io.open(filename, mode=mode+'b')
    else:
        return io.open(filename, mode=mode, newline='')
    

def main(bvhFilePath: str, rotation: bool = False):

    if not os.path.exists(bvhFilePath):
        print(f"Error: file {bvhFilePath} not found.")
        sys.exit(0)

    print(f"Input filename: {bvhFilePath}")

    other_s = process_bvhfile(bvhFilePath)

    print(f"Analyzing frames...")
    for i in range(other_s.frames):
        new_frame = process_bvhkeyframe(other_s.keyframes[i], 
                                        other_s.root,
                                        other_s.dt * i)
    print(f"done")
    
    positions_StartTime = time()

    file_out = bvhFilePath[:-4] + "_positions.csv"

    with open_csv(file_out, 'w') as f:
        writer = csv.writer(f)
        header, frames = other_s.get_frames_worldpos()
        writer.writerow(header)
        for frame in frames:
            writer.writerow(frame)
    print(f"World Positions Output file: {file_out}")

    positions_endTime = time()

    print(f"positions: {positions_endTime - positions_StartTime}s")

    rotations_StartTime = time()

    if rotation:
        file_out = bvhFilePath[:-4] + "_rotations.csv"
    
        with open_csv(file_out, 'w') as f:
            writer = csv.writer(f)
            header, frames = other_s.get_frames_rotations()
            writer.writerow(header)
            for frame in frames:
                writer.writerow(frame)
        print(f"Rotations Output file: {file_out}")

    rotations_endTime = time()

    print(f"rotations: {rotations_endTime - rotations_StartTime}s")

if __name__ == "__main__":
    main("test/ex/PT_S104_1_FM_M_009.bvh", True)
