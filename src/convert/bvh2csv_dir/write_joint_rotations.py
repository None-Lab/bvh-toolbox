from time import time

import os

import numpy as np
import transforms3d as t3d

from multiprocessing import Pool

from ... import BvhTree
from ... import get_affines

def write_joint_rotations(bvh_tree, filepath):
    """관절의 회전 데이터를 CSV 파일로 씁니다.

    :param bvh_tree: BVH 트리가 데이터를 보유합니다.
    :type bvh_tree: BvhTree
    :param filepath: CSV 파일의 대상 파일 경로.
    :type filepath: str
    :return: 쓰기 프로세스가 성공했는지 여부.
    :rtype: bool
    """
    startTime = time()
    
    time_col = np.arange(0, (bvh_tree.nframes - 0.5)*bvh_tree.frame_time, bvh_tree.frame_time)[:, None]
    data_list = [time_col]
    header = ['time']
    for joint in bvh_tree.get_joints():
        channels = [channel for channel in bvh_tree.joint_channels(joint.name) if channel[1:] == 'rotation']
        header.extend(['{}.{}'.format(joint.name, channel[:1].lower()) for channel in channels])
        data_list.append(np.array(bvh_tree.frames_joint_channels(joint.name, channels)))
        
    data = np.concatenate(data_list, axis=1)

    endTime = time()
    print(f"rotations: {endTime - startTime}s")

    try:
        np.savetxt(filepath, data, header=','.join(header), fmt='%10.5f', delimiter=',', comments='')
        return True
    except IOError as e:
        print("ERROR({}): Could not write to file {}.\n"
              "Make sure you have writing permissions.\n".format(e.errno, filepath))
        return False
