from time import time

import os

import numpy as np
import transforms3d as t3d

from multiprocessing import Pool

from ... import BvhTree
from ... import get_affines

def write_joint_hierarchy(bvh_tree, filepath, scale=1.0):
    """ 관절의 세계 좌표 위치 데이터를 CSV 파일로 씁니다.

    :param bvh_tree: BVH 트리가 데이터를 보유합니다.
    :type bvh_tree: BvhTree
    :param filepath: CSV 파일의 대상 파일 경로.
    :type filepath: str
    :param scale: 오프셋 값에 대한 배율.
    :type scale: float
    :return: 쓰기 프로세스가 성공했는지 여부.
    :rtype: bool
    """
    startTime = time()

    data = list()
    for joint in bvh_tree.get_joints(end_sites=True):
        
        joint_name = joint.name
        
        parent_name = bvh_tree.joint_parent(joint_name).name if bvh_tree.joint_parent(joint_name) else ''

        row = [joint_name, parent_name]
        
        row.extend((scale * offset for offset in bvh_tree.joint_offset(joint.name)))
        
        data.append(tuple(row))

    data = np.array(data, dtype=[('joint', np.unicode_, 20),
                                 ('parent', np.unicode_, 20),
                                 ('offset.x', float),
                                 ('offset.y', float),
                                 ('offset.z', float)])
    
    endTime = time()
    print(f"hierarchy: {endTime - startTime}s")

    try:
        np.savetxt(filepath,
                   data,
                   header=','.join(data.dtype.names),
                   fmt=['%s', '%s', '%10.5f', '%10.5f', '%10.5f'],
                   delimiter=',',
                   comments='')
        return True
    except IOError as e:
        print("ERROR({}): Could not write to file {}.\n"
              "Make sure you have writing permissions.\n".format(e.errno, filepath))
        return False
        pos_processing = Pool(Processing_CPU_count).apply_async(write_joint_positions, (mocap, f'{dst_filepath}_pos.csv', scale, end_sites, ))
