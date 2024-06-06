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

""" BVH 파일의 애니메이션 데이터를 CSV 형식으로 변환합니다.
회전 데이터와 위치는 별도의 출력 파일입니다.
첫 번째 줄은 각 자유도를 열로 하는 헤더입니다.
첫 번째 열은 해당 행의 데이터에 대한 프레임입니다.
두 번째 열은 프레임의 시간(초)입니다.
조인트의 자유도는 BVH 파일에서 채널로 표시되는 순서대로 작성됩니다.
"""

import os

import numpy as np
import transforms3d as t3d

from .. import BvhTree
from .. import get_affines


def write_joint_rotations(bvh_tree, filepath):
    """관절의 회전 데이터를 CSV 파일로 씁니다.

    :param bvh_tree: BVH 트리가 데이터를 보유합니다.
    :type bvh_tree: BvhTree
    :param filepath: CSV 파일의 대상 파일 경로.
    :type filepath: str
    :return: 쓰기 프로세스가 성공했는지 여부.
    :rtype: bool
    """
    
    time_col = np.arange(0, (bvh_tree.nframes - 0.5)*bvh_tree.frame_time, bvh_tree.frame_time)[:, None]
    data_list = [time_col]
    header = ['time']
    for joint in bvh_tree.get_joints():
        channels = [channel for channel in bvh_tree.joint_channels(joint.name) if channel[1:] == 'rotation']
        header.extend(['{}.{}'.format(joint.name, channel[:1].lower()) for channel in channels])
        data_list.append(np.array(bvh_tree.frames_joint_channels(joint.name, channels)))
        
    data = np.concatenate(data_list, axis=1)
    try:
        np.savetxt(filepath, data, header=','.join(header), fmt='%10.5f', delimiter=',', comments='')
        return True
    except IOError as e:
        print("ERROR({}): Could not write to file {}.\n"
              "Make sure you have writing permissions.\n".format(e.errno, filepath))
        return False
    

def write_joint_positions(bvh_tree, filepath, scale=1.0, end_sites=False):
    """관절의 세계 좌표 위치 데이터를 CSV 파일로 씁니다.

    :param bvh_tree: BVH 트리가 데이터를 보유합니다.
    :type bvh_tree: BvhTree
    :param filepath: CSV 파일의 대상 파일 경로.
    :type filepath: str
    :param scale: 루트 위치 및 오프셋 값에 대한 배율.
    :type scale: float
    :param end_sites: 위치 CSV에 BVH 종단 지점 포함.
    :type end_sites: bool
    :return: 쓰기 프로세스가 성공했는지 여부.
    :rtype: bool
    """
    
    time_col = np.arange(0, (bvh_tree.nframes - 0.5) * bvh_tree.frame_time, bvh_tree.frame_time)[:, None]
    data_list = [time_col]
    header = ['time']
    root = next(bvh_tree.root.filter('ROOT'))
    
    def get_world_positions(joint):
        if joint.value[0] == 'End':
            joint.world_transforms = np.tile(t3d.affines.compose(np.zeros(3), np.eye(3), np.ones(3)),
                                             (bvh_tree.nframes, 1, 1))
        else:
            channels = bvh_tree.joint_channels(joint.name)
            axes_order = ''.join([ch[:1] for ch in channels if ch[1:] == 'rotation']).lower()  # FixMe: This isn't going to work when not all rotation channels are present
            axes_order = 's' + axes_order[::-1]
            joint.world_transforms = get_affines(bvh_tree, joint.name, axes=axes_order)
            
        if joint != root:
            # For joints substitute position for offsets.
            offset = [float(o) for o in joint['OFFSET']]
            joint.world_transforms[:, :3, 3] = offset
            joint.world_transforms = np.matmul(joint.parent.world_transforms, joint.world_transforms)
        if scale != 1.0:
            joint.world_transforms[:, :3, 3] *= scale
            
        header.extend(['{}.{}'.format(joint.name, channel) for channel in 'xyz'])
        pos = joint.world_transforms[:, :3, 3]
        data_list.append(pos)
        
        if end_sites:
            end = list(joint.filter('End'))
            if end:
                get_world_positions(end[0])  # There can be only one End Site per joint.
        for child in joint.filter('JOINT'):
            get_world_positions(child)
    
    get_world_positions(root)
    data = np.concatenate(data_list, axis=1)
    try:
        np.savetxt(filepath, data, header=','.join(header), fmt='%10.5f', delimiter=',', comments='')
        return True
    except IOError as e:
        print("ERROR({}): Could not write to file {}.\n"
              "Make sure you have writing permissions.\n".format(e.errno, filepath))
        return False


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


def bvh2csv(bvh_path,
            dst_dirpath=None,
            scale=1.0,
            export_rotation=True,
            export_position=True,
            export_hierarchy=True,
            end_sites=True):
    """BVH 파일을 CSV 파일 형식으로 변환합니다.
    키워드 인수를 전달할 때는 키워드를 사용해야 합니다!

    :param bvh_path: BVH 소스 파일 경로(들).
    :type bvh_path: str|list
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
        with open(bvh_path) as file_handle:
            mocap = BvhTree(file_handle.read())
    except IOError as e:
        print("ERROR {}: Could not open file".format(e.errno), bvh_path)
        return False

    # Assume everything works and only set False on error.
    pos_success = True
    rot_success = True
    hierarchy_success = True
    if not dst_dirpath:
        dst_filepath = os.path.join(os.path.dirname(bvh_path), os.path.basename(bvh_path)[:-4])
    else:
        # If the specified path doesn't yet exist, create it.
        if not os.path.exists(dst_dirpath):
            os.mkdir(dst_dirpath)
        dst_filepath = os.path.join(dst_dirpath, os.path.basename(bvh_path)[:-4])
    if export_position:
        pos_success = write_joint_positions(mocap, dst_filepath + '_pos.csv', scale, end_sites)
    if export_rotation:
        rot_success = write_joint_rotations(mocap, dst_filepath + '_rot.csv')
    if export_hierarchy:
        hierarchy_success = write_joint_hierarchy(mocap, dst_filepath + '_hierarchy.csv', scale)

    n_succeeded = sum([pos_success, rot_success, hierarchy_success])
    return bool(n_succeeded)
