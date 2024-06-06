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

""" CSV 파일을 BVH 파일로 변환합니다.
현재는 3개의 CSV 파일이 필요합니다.
* 계층 구조 파일
* 회전 파일
* 위치 파일
이론적으로 계층 구조와 위치 데이터만으로도 bvh 파일을 재구성할 수 있습니다.
하지만 급각과 둔각 문제를 해결하고 이를 오일러 회전 순서로 분해해야 합니다.
시간이 없어요! :D
"""

import os
import errno

import numpy as np

from .. import BvhTree


def get_hierarchy_data(hierarchy_file, scale=1.0):
    """ CSV 파일에서 스켈레톤 계층 구조 데이터를 가져옵니다.
    
    :param hierarchy_file: 계층 구조가 포함된 CSV 소스 파일입니다. 열: joint, parent, offset.x, offset.y, offset.z
    :type hierarchy_file: str
    :param scale: 오프셋 값에 대한 배율입니다.
    :type scale: float
    :return: 스켈레톤 계층 구조가 있는 딕셔너리입니다. {이름: {'부모': str, '오프셋': 배열, '자식': 목록}}
    :rtype: dict
    """
    try:
        rec_array = np.recfromcsv(hierarchy_file, encoding='UTF-8')
    except OSError as e:
        print("ERROR:", e)
        raise
    offsets = np.array(rec_array[['offsetx', 'offsety', 'offsetz']].tolist())
    offsets *= scale
    joint_info = {j[0]: {'parent': j[1], 'offset': offsets[i], 'children': []}
                  for i, j in enumerate(rec_array)}
    _update_children(joint_info)
    input_is_sane = hierarchy_sanity_check(joint_info)
    return joint_info
    

def _update_children(nodes):
    """ 조인트의 children 속성을 채웁니다.
    
    :param nodes: 스켈레톤 계층 구조가 담긴 딕셔너리입니다.
    :type nodes: dict
    """
    for node, properties in nodes.items():
        parent = properties['parent']
        try:
            p_props = nodes[parent]
        except KeyError:
            if not parent:  # Root has no parent.
                continue
            raise ValueError("ERROR: Parent of {} cannot be found in the hierarchy definition!".format(node))
        p_children = p_props['children']
        p_children.append(node)
        p_props.update({'children': p_children})
        
        
def _update_channels(nodes, root_pos_channels, rot_channels):
    """ 노드에 루트 위치 채널과 조인트 회전 채널 정보를 추가합니다.
    
    :param nodes: 업데이트할 스켈레톤 계층 구조가 담긴 딕셔너리입니다.
    :param root_pos_channels: 루트가 갖는 위치 채널입니다.
    :type root_pos_channels: list
    :param rot_channels: 조인트 이름과 해당 회전 채널 목록의 매핑이 담긴 딕셔너리입니다.
    :type rot_channels: dict
    """
    for node, properties in nodes.items():
        # End sites do not have any channels.
        if not properties['children']:
            continue
        if not properties['parent']:
            properties['channels'] = root_pos_channels + rot_channels[node]
        else:
            properties['channels'] = rot_channels[node]


def hierarchy_sanity_check(nodes):
    """ 스켈레톤 계층 구조에서 오류를 확인합니다.
    
    :param nodes: 스켈레톤 계층 구조가 담긴 딕셔너리입니다.
    :type nodes: dict
    :return: 오류가 발생하지 않으면 True를 반환합니다.
    :rtype: bool
    """
    found_root = False
    for node, properties in nodes.items():
        parent = properties['parent']
        if not parent:
            if found_root:
                raise ValueError("ERROR: Hierarchy can't have more than 1 root!")
            else:
                found_root = True
        if node == parent:
            raise ValueError("ERROR: Joint {} cannot be parent of itself!".format(node))
        if parent in properties['children']:
            raise ValueError("Impossible cyclic relation detected for joints {} and {}!".format(node, parent))
        if parent and parent not in nodes.keys():
            raise ValueError("ERROR: Parent of joint {} cannot be found in the hierarchy definition!".format(node))
    if not found_root:  # Would probably be caught before.
        raise ValueError("ERROR: No root joint found in the hierarchy definition!")
        
    # No error occurred.
    return True
    

def get_root_name(nodes):
    """ 계층 구조의 루트를 찾습니다.
    
    :param nodes: 스켈레톤 계층 구조가 담긴 딕셔너리입니다.
    :type nodes: dict
    :return: 루트 조인트의 이름입니다.
    :rtype: str
    """
    for joint, properties in nodes.items():
        if not properties['parent']:
            return joint
    return None  # Should never be reached.

    
def get_transform_data(transforms_file):
    """ 변환 파일에서 열 이름과 데이터를 리스트와 넘파이 배열로 반환합니다.
    
    :param transforms_file: 회전 또는 위치와 같은 변환 데이터가 포함된 CSV 소스 파일입니다.
    :type: str
    :return: 열 이름 리스트와 데이터입니다.
    :rtype: tuple
    """
    # Numpy is losing case of column names when using np.recfromcsv().
    # Read 1st line as header manually instead and use genfromtxt() to get data.
    try:
        data = np.genfromtxt(transforms_file, delimiter=",", skip_header=1)
    except OSError as e:
        print("ERROR:", e)
        raise
    with open(transforms_file, 'rb') as file_handle:
        columns = file_handle.readline().strip().decode("utf-8").split(sep=',')
        
    return columns, data
    

def df_to_joints(df):
    """ 자유도 목록에서 조인트 이름을 반환합니다. 조인트 이름 목록은 알파벳 순으로 정렬됩니다.
    
    :param df: "joint.x"와 같은 자유도 목록입니다.
    :type df: list
    :return: 조인트 이름입니다.
    :rtype: list
    """
    joints = sorted(set((x.split('.')[0] for x in df if x != 'time')))
    return joints


def _df_to_channels(df):
    """ 자유도 목록에서 조인트 이름과 해당 자유도를 매핑합니다.
    
    :param df: 회전 정보가 있는 rotations.csv 파일의 열 이름 목록입니다.
    :type df: list
    :return: 조인트 이름을 키로, 해당 자유도 목록을 값으로 갖는 딕셔너리입니다.
    :rtype: dict
    """
    # Get a version of df that does not contain 'time'.
    dof = df.copy()
    dof.remove('time')
    joints = df_to_joints(df)
    joint_channels = dict()
    try:
        for joint in joints:
            joint_channels[joint] = [ch.split('.')[1].upper() + "rotation" for ch in dof if joint == ch.split('.')[0]]
    except IndexError:
        raise ValueError("ERROR: degrees of freedom (columns) must be in the form of: joint.axis, e.g. Hips.x")
    return joint_channels
    
    
def _get_frame_time(time_steps):
    """ 시간 단계의 평균 프레임 시간을 계산합니다.
    
    :param time_steps: 누적 프레임 시간이 있는 1D 배열입니다.
    :type time_steps: numpy.ndarray
    :return: 각 프레임의 평균 길이(초 단위).
    :rtype: float
    """
    if len(time_steps.shape) != 1:
        raise ValueError("ERROR: Time series must be a 1D array.")
    frame_time = time_steps[-1]/(len(time_steps) - 1)  # Need to ignore the first frame (0).
    return frame_time

    
def _get_joint_depth(nodes, joint):
    """ 계층 구조에서 해당 조인트의 깊이를 반환합니다.
    
    :param nodes: 스켈레톤 계층 구조가 담긴 딕셔너리입니다.
    :type nodes: dict
    :param joint: 해당 조인트의 이름입니다.
    :type joint: str
    :return: 계층 구조에서 조인트의 깊이를 나타내는 정수입니다.
    :rtype: int
    """
    depth = 0
    parent = nodes[joint]['parent']
    while parent:
        depth += 1
        parent = nodes[parent]['parent']
    return depth


def _get_joint_string(nodes, joint):
    """
    계층 구조에서 조인트의 bvh 문자열 표현을 조립합니다.
    조인트의 깊이에 따라 들여쓰기가 적용됩니다.

    :param nodes: 스켈레톤 계층 구조가 담긴 딕셔너리입니다.
    :type nodes: dict
    :param joint: 해당 조인트의 이름입니다.
    :type joint: str
    :return: bvh 계층 구조 섹션의 조인트 부분입니다.
    :rtype: str
    """
    depth = _get_joint_depth(nodes, joint)
    properties = nodes[joint]
    if not properties['parent']:
        s = '{0}ROOT {1}\n'.format('  ' * depth, str(joint))
    elif not properties['children']:
        s = '{0}{1}\n'.format('  ' * depth, 'End Site')
    else:
        s = '{0}JOINT {1}\n'.format('  ' * depth, str(joint))
        
    s += '{0}{{\n'.format('  ' * depth)
    s += '{0}{1} {2}\n'.format('  ' * (depth + 1), 'OFFSET', ' '.join(properties['offset'].astype(str)))
    if not properties['children']:
        s += '{0}}}\n'.format('  ' * depth)
    else:
        s += '{0}{1} {2} {3}\n'.format('  ' * (depth + 1), 'CHANNELS',
                                       len(properties['channels']),
                                       ' '.join(properties['channels']))
    return s


def _close_scopes(hierarchy_string, target_depth=0):
    """ The string is hierarchically ordered. This function appends curly brackets to close open scopes.
    It takes the indentation of the last closed bracket as reference for the current level/depth of the hierarchy.
    :param hierarchy_string:
    :type hierarchy_string: str
    :param target_depth: The depth determines the target indentation.
    :type target_depth: int
    :return: string with closed scopes.
    :rtype: str
    """
    # Get the last level by counting the spaces to the second to last line break.
    last_depth = hierarchy_string[hierarchy_string[:-1].rfind("\n"):].count("  ")
    diff = last_depth - target_depth
    for depth in range(diff):
        hierarchy_string += '{0}}}\n'.format('  ' * (last_depth - depth - 1))
    return hierarchy_string


def _get_hierarchy_string(nodes):
    """ Compose the hierarchy part of a bvh file.
    
    :param nodes: Dictionary with skeleton hierarchy.
    :type nodes: dict
    :return: Hierarchy as bvh string representation.
    :rtype: str
    """
    s = 'HIERARCHY\n'
    for joint in nodes:
        s = _close_scopes(s, _get_joint_depth(nodes, joint))
        s += _get_joint_string(nodes, joint)
    s = _close_scopes(s)
    return s


def _get_motion_string(n_frames, frame_time, frames):
    """ Compose the motion part of a bvh file.
    
    :param n_frames: Number of frames.
    :type n_frames: int
    :param frame_time: Time in seconds it takes to advance 1 frame.
    :type frame_time: float
    :param frames: The motion data for channels of all joints.
    :type frames: numpy.ndarray
    :return: Motion as string representation.
    :rtype: str
    """
    s = 'MOTION\n'
    s += 'Frames: {}\n'.format(n_frames)
    s += 'Frame Time: {}\n'.format(frame_time)
    for frame in frames.astype(str):
        s += ' '.join(frame)
        s += '\n'
    return s
    

def csv2bvh_string(hierarchy_file, position_file, rotation_file, scale=1.0):
    """ Compose the contents of a bvh file from CSV files.
    
    :param hierarchy_file: CSV file path containing hierarchy. Columns: joint, parent, offset.x, offset.y, offset.z
    :type hierarchy_file: str
    :param position_file: CSV file path to positions.
    :type position_file: str
    :param rotation_file:CSV file path to rotations.
    :type rotation_file: str
    :param scale: Scale factor for root position and offset values.
    :type scale: float
    :return: String that can be written to BVH file or build BvhTree from.
    :rtype: str
    """
    joint_info = get_hierarchy_data(hierarchy_file)
    
    # Get root positions.
    root_name = get_root_name(joint_info)
    pos_df, positions = get_transform_data(position_file)
    positions *= scale
    root_pos_df = [channel for channel in pos_df if root_name == channel.split('.')[0]]
    if not root_pos_df:  # Make sure root is in position data.
        raise Exception("ERROR: No position data found in {} for hierarchy's root '{}'.\n"
                        "Make sure the names match. They are case-sensitive.".format(position_file, root_name))
    root_pos_channels = [channel.split('.')[1].upper() + "position" for channel in root_pos_df]
    root_pos_indices = [pos_df.index(df) for df in root_pos_df]  # Get data indices for root position df.
    root_pos_data = positions[:, root_pos_indices]
    n_frames = len(root_pos_data)
    
    # Get joint rotations.
    rot_df, rotations = get_transform_data(rotation_file)
    # Make sure there are as many frames as in position data.
    if len(rotations) != n_frames:
        raise Exception("ERROR: Mismatch of frame count between positions and rotations!")
    rot_joint_names = df_to_joints(rot_df)
    
    # Make sure there's rotation data for each joint that is not an end site.
    non_leaf_nodes = sorted([joint for joint in joint_info if joint_info[joint]['children']])
    if rot_joint_names != non_leaf_nodes:
        missing_joint_data = set(non_leaf_nodes).difference(rot_joint_names)
        raise Exception("ERROR: No rotation data found for: {}".format(", ".join(missing_joint_data)))
    
    # Make HIERARCHY section.
    rot_channels = _df_to_channels(rot_df)
    _update_channels(joint_info, root_pos_channels, rot_channels)
    hierarchy_string = _get_hierarchy_string(joint_info)
    
    # Make MOTION section.
    # Let's just assume the degrees of freedom are in the correct order (successive) for now.
    # Ignore time column (index=0). # Todo: Don't assume.
    motion_data = np.concatenate((root_pos_data, rotations[:, 1:]), axis=1)
    frame_time = _get_frame_time(rotations[:, rot_df.index('time')])
    motion_string = _get_motion_string(n_frames, frame_time, motion_data)
    
    bvh_string = hierarchy_string + motion_string
    return bvh_string
    

def csv2bvhtree(hierarchy_file, position_file, rotation_file, scale=1.0):
    """ Get a BvhTree instance from input CSV files.

    :param hierarchy_file: CSV file path containing hierarchy. Columns: joint, parent, offset.x, offset.y, offset.z
    :type hierarchy_file: str
    :param position_file: CSV file path to positions.
    :type position_file: str
    :param rotation_file:CSV file path to rotations.
    :type rotation_file: str
    :param scale: Scale factor for root position and offset values.
    :type scale: float
    :return: BVHTree instance.
    :rtype: bvh.BvhTree
    """
    data = csv2bvh_string(hierarchy_file, position_file, rotation_file, scale)
    bvh_tree = BvhTree(data)
    return bvh_tree


def write(data, out_stream):
    out_stream.write(data)


def write_file(data, file_path):
    """ Write data to file.
    
    :param data: Content to write to file.
    :type data: str
    :param file_path: Destination path for BVH file.
    :type file_path: str
    :return: If writing to file was possible.
    :rtype: bool
    """
    try:
        with open(file_path, "w") as file_handle:
            write(data, file_handle)
        return True
    except OSError as e:
        print("ERROR:", e)
        return False
    

def csv2bvh(hierarchy_file, position_file, rotation_file, destination_file=None, scale=1.0):
    """ Composes BVH file from CSV input files. If no destination path is given, CSV file path is used.
    
    :param hierarchy_file: CSV file path containing hierarchy. Columns: joint, parent, offset.x, offset.y, offset.z
    :type hierarchy_file: str
    :param position_file: CSV file path to positions.
    :type position_file: str
    :param rotation_file:CSV file path to rotations.
    :type rotation_file: str
    :param destination_file: Output BVH file path.
    :type destination_file: str
    :param scale: Scale factor for root position and offset values.
    :type scale: float
    :return: Whether writing to file was successful or not.
    """
    if not destination_file:
        bvh_file = os.path.basename(hierarchy_file).replace('_hierarchy', '').replace('.csv', '.bvh')
        destination_file = os.path.join(os.path.dirname(hierarchy_file), bvh_file)
    else:
        # If the specified path doesn't yet exist, create it.
        if not os.path.exists(os.path.dirname(destination_file)):
            try:
                os.mkdir(os.path.dirname(destination_file))
            except OSError as exc:  # Guard against race condition.
                if exc.errno != errno.EEXIST:
                    raise

    data = csv2bvh_string(hierarchy_file, position_file, rotation_file, scale)
    success = write_file(data, destination_file)
    return success
