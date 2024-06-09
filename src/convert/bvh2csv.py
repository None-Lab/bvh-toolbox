# MIT License
#
# Copyright (c) 2018 Olaf Haag
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from time import time

import os

import numpy as np
import transforms3d as t3d

from multiprocessing import Pool

from .. import BvhTree
from .. import get_affines

from .bvh2csv_dir import (
    write_joint_hierarchy,
    write_joint_positions,
    write_joint_rotations
)


def bvh2csv(bvh_path: str, dst_dirpath=None, scale=1.0,
            export_rotation=True, export_position=True, export_hierarchy=True,
            end_sites=True):
    """BVH 파일을 CSV 파일 형식으로 변환합니다.
    키워드 인수를 전달할 때는 키워드를 사용해야 합니다!

    :param bvh_path: BVH 소스 파일 경로.
    :type bvh_path: str
    :param dst_dirpath: 대상 CSV 파일의 폴더 경로.
    :type dst_dirpath: str
    :param scale: 루트 위치 및 오프셋 값에 대한 배율.
    :type scale: float
    :param export_rotation: 회전 CSV 파일 출력.
    :type export_rotation: bool
    :param export_position: 월드 공간 위치 CSV 파일 출력.
    :type export_position: bool
    :param export_hierarchy: 계층 구조 CSV 파일 출력.
    :type export_hierarchy: bool
    :param end_sites: 위치 CSV에 BVH 종단 지점 포함.
    :type end_sites: bool
    :return: 변환이 성공했는지 여부.
    :rtype: bool
    """
    try:
        startTime = time()
        
        with open(bvh_path) as file_handle:
            mocap = BvhTree(file_handle.read())
        
        endTime = time()
        print(f"file read: {endTime - startTime}s")

    except IOError as e:
        print("ERROR {}: Could not open file".format(e.errno), bvh_path)
        return False

    # Assume everything works and only set False on error.
    pos_success = False
    rot_success = False
    hierarchy_success = False
    
    if not dst_dirpath:
        dst_filepath = os.path.join(os.path.dirname(bvh_path), os.path.basename(bvh_path)[:-4])
    
    else:
        # If the specified path doesn't yet exist, create it.
        if not os.path.exists(dst_dirpath):
            os.mkdir(dst_dirpath)
    
        dst_filepath = os.path.join(dst_dirpath, os.path.basename(bvh_path)[:-4])
    

    if export_position:
        pos_processing = Pool(1).apply_async(write_joint_positions, (mocap, f'{dst_filepath}_pos.csv', scale, end_sites, ))
        pos_success = pos_processing.get()

    if export_rotation:
        rot_processing = Pool(1).apply_async(write_joint_rotations, (mocap, f'{dst_filepath}_rot.csv', ))
        rot_success = rot_processing.get()

    if export_hierarchy:
        hierarchy_processing = Pool(1).apply_async(write_joint_hierarchy, (mocap, f'{dst_filepath}_hierarchy.csv', scale, ))
        hierarchy_success = hierarchy_processing.get()
    
    return bool(pos_success and rot_success and hierarchy_success)
