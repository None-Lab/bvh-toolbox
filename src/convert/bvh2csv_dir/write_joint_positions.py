from time import time
import os
import numpy as np
import transforms3d as t3d
from multiprocessing import Pool

from ... import BvhTree
from ... import get_affines

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
    
    joint_stack = [root]

    while joint_stack:
        joint = joint_stack.pop()
        
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
                # End Site를 스택에 추가
                joint_stack.append(end[0])
        
        # 자식 노드를 스택에 추가
        for child in joint.filter('JOINT'):
            joint_stack.append(child)
    

    data = np.concatenate(data_list, axis=1)    

    try:
        np.savetxt(filepath, data, header=','.join(header), fmt='%10.5f', delimiter=',', comments='')
        return True

    except IOError as e:
        print(f"""ERROR({e.errno}): Could not write to file {filepath}.\n
                  Make sure you have writing permissions.\n""")
        return False