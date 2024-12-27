import os
import sys

cwd = os.getcwd()
sys.path.append(cwd)
sys.path.append(cwd + '/src/simulator/utils')
sys.path.append(cwd + '/src/simulator')
sys.path.append(cwd + '/build/lib')

from collections import OrderedDict

import pybullet as p
import numpy as np
from tqdm import tqdm
import cv2
import imageio

import utils.util as util
import utils.liegroup


def get_robot_config(robot,
                     b_print_info=False):
    nq, nv, na, joint_id, link_id = 0, 0, 0, OrderedDict(), OrderedDict()
    link_id[(p.getBodyInfo(robot)[0]).decode("utf-8")] = -1
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        if info[2] != p.JOINT_FIXED:
            joint_id[info[1].decode("utf-8")] = info[0]
        link_id[info[12].decode("utf-8")] = info[0]
        nq = max(nq, info[3])
        nv = max(nv, info[4])
    nq += 1
    nv += 1
    na = len(joint_id)

    if b_print_info:
        print("=" * 80)
        print("SimulationRobot")
        print("nq: ", nq, ", nv: ", nv, ", na: ", na)
        print("+" * 80)
        print("Joint Infos")
        util.PrettyPrint(joint_id)
        print("+" * 80)
        print("Link Infos")
        util.PrettyPrint(link_id)

    return nq, nv, na, joint_id, link_id


def get_kinematics_config(robot, joint_id, link_id, open_chain_joints,
                          base_link, ee_link):
    joint_screws_in_ee = np.zeros((6, len(open_chain_joints)))
    ee_link_state = p.getLinkState(robot, link_id[ee_link], 1, 1)
    if link_id[base_link] == -1:
        base_pos, base_quat = p.getBasePositionAndOrientation(robot)
    else:
        base_link_state = p.getLinkState(robot, link_id[base_link], 1, 1)
        base_pos, base_quat = base_link_state[0], base_link_state[1]
    T_w_b = liegroup.RpToTrans(util.quat_to_rot(np.array(base_quat)),
                               np.array(base_pos))
    T_w_ee = liegroup.RpToTrans(util.quat_to_rot(np.array(ee_link_state[1])),
                                np.array(ee_link_state[0]))
    T_b_ee = np.dot(liegroup.TransInv(T_w_b), T_w_ee)
    for i, joint_name in enumerate(open_chain_joints):
        joint_info = p.getJointInfo(robot, joint_id[joint_name])
        link_name = joint_info[12].decode("utf-8")
        joint_type = joint_info[2]
        joint_axis = joint_info[13]
        screw_at_joint = np.zeros(6)
        link_state = p.getLinkState(robot, link_id[link_name], 1, 1)
        T_w_j = liegroup.RpToTrans(util.quat_to_rot(np.array(link_state[5])),
                                   np.array(link_state[4]))
        T_ee_j = np.dot(liegroup.TransInv(T_w_ee), T_w_j)
        Adj_ee_j = liegroup.Adjoint(T_ee_j)
        if joint_type == p.JOINT_REVOLUTE:
            screw_at_joint[0:3] = np.array(joint_axis)
        elif joint_type == p.JOINT_PRISMATIC:
            screw_at_joint[3:6] = np.array(joint_axis)
        else:
            raise ValueError
        joint_screws_in_ee[:, i] = np.dot(Adj_ee_j, screw_at_joint)

    return joint_screws_in_ee, T_b_ee


def get_link_iso(robot, link_idx):
    info = p.getLinkState(robot, link_idx, 1, 1)
    pos = np.array(info[0])
    rot = util.quat_to_rot(np.array(info[1]))

    return liegroup.RpToTrans(rot, pos)


def get_link_vel(robot, link_idx):
    info = p.getLinkState(robot, link_idx, 1, 1)
    ret = np.zeros(6)
    ret[3:6] = np.array(info[6])
    ret[0:3] = np.array(info[7])

    return ret


def set_link_damping(robot, link_id, lin_damping, ang_damping):
    for i in link_id:
        p.changeDynamics(robot,
                         i,
                         linearDamping=lin_damping,
                         angularDamping=ang_damping)


def set_joint_friction(robot, joint_id, max_force=0):
    p.setJointMotorControlArray(robot, [*joint_id.values()],
                                p.VELOCITY_CONTROL,
                                forces=[max_force] * len(joint_id))


def draw_link_frame(robot, link_idx, linewidth=5.0, text=None):
    # This only works when the link has an visual element defined in the urdf file
    if text is not None:
        p.addUserDebugText(text, [0, 0, 0.1],
                           textColorRGB=[1, 0, 0],
                           textSize=1.5,
                           parentObjectUniqueId=robot,
                           parentLinkIndex=link_idx)

    p.addUserDebugLine([0, 0, 0], [0.1, 0, 0], [1, 0, 0],
                       linewidth,
                       parentObjectUniqueId=robot,
                       parentLinkIndex=link_idx)

    p.addUserDebugLine([0, 0, 0], [0, 0.1, 0], [0, 1, 0],
                       linewidth,
                       parentObjectUniqueId=robot,
                       parentLinkIndex=link_idx)

    p.addUserDebugLine([0, 0, 0], [0, 0, 0.1], [0, 0, 1],
                       linewidth,
                       parentObjectUniqueId=robot,
                       parentLinkIndex=link_idx)


def set_motor_impedance(robot, joint_id, command, kp, kd):
    trq_applied = OrderedDict()
    for (joint_name, pos_des), (_, vel_des), (_, trq_des) in zip(
            command['joint_pos'].items(), command['joint_vel'].items(),
            command['joint_trq'].items()):
        joint_state = p.getJointState(robot, joint_id[joint_name])
        joint_pos, joint_vel = joint_state[0], joint_state[1]
        trq_applied[joint_id[joint_name]] = trq_des + kp[joint_name] * (
            pos_des - joint_pos) + kd[joint_name] * (vel_des - joint_vel)

    p.setJointMotorControlArray(robot,
                                trq_applied.keys(),
                                controlMode=p.TORQUE_CONTROL,
                                forces=list(trq_applied.values()))


def set_motor_trq(robot, joint_id, trq_cmd):
    trq_applied = OrderedDict()
    if(isinstance(trq_cmd, np.ndarray)):
        for joint_id, trq_des in enumerate(trq_cmd):
            trq_applied[joint_id] = trq_des
    else:
        for joint_name, trq_des in trq_cmd.items():
            trq_applied[joint_id[joint_name]] = trq_des

    p.setJointMotorControlArray(robot,
                                trq_applied.keys(),
                                controlMode=p.TORQUE_CONTROL,
                                forces=list(trq_applied.values()))


def set_motor_pos(robot, joint_id, pos_cmd):
    pos_applied = OrderedDict()
    if(isinstance(pos_cmd, np.ndarray)):
        for joint_id, pos_des in enumerate(pos_cmd):
            pos_applied[joint_id] = pos_des
    else:
        for joint_name, pos_des in pos_cmd.items():
            pos_applied[joint_id[joint_name]] = pos_des
            
    p.setJointMotorControlArray(robot,
                                pos_applied.keys(),
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=list(pos_applied.values()))


def set_motor_pos_vel(robot, joint_id, pos_cmd, vel_cmd):
    pos_applied = OrderedDict()
    vel_applied = OrderedDict()

    if(isinstance(pos_cmd, np.ndarray)):
        for joint_id, pos_des in enumerate(pos_cmd):
            pos_applied[joint_id] = pos_des
    else:
        for joint_name, pos_des in pos_cmd.items():
            pos_applied[joint_id[joint_name]] = pos_des
            
    if(isinstance(vel_cmd, np.ndarray)):
        for joint_id, vel_des in enumerate(vel_cmd):
            vel_applied[joint_id] = vel_des
    else:
        for joint_name, vel_des in vel_cmd.items():
            vel_applied[joint_id[joint_name]] = vel_des
            
    p.setJointMotorControlArray(robot,
                                pos_applied.keys(),
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=list(pos_applied.values()),
                                targetVelocities=list(vel_applied.values()))


def get_sensor_data(robot, joint_id):
    """
    Parameters
    ----------
    joint_id (dict):
        Joint ID Dict
    Returns
    -------
    sensor_data (dict):
        joint_pos (dict):
            Joint pos
        joint_vel (dict):
            Joint vel
    """
    sensor_data = OrderedDict()

    # Joint Quantities
    sensor_data['joint_pos'] = list()
    sensor_data['joint_vel'] = list()
    for k, v in joint_id.items():
        js = p.getJointState(robot, v)
        sensor_data['joint_pos'].append( js[0] )
        sensor_data['joint_vel'].append( js[1] )

    return sensor_data

def enable_trq_data(robot, joint_id):
    for k, v in joint_id.items():
        p.enableJointForceTorqueSensor(robot, v)

def get_trq_data(robot, joint_id, sensor_data):
    sensor_data['joint_trq'] = list()
    for k, v in joint_id.items():
        trq =p.getJointState(robot, v)[2]
        sensor_data['joint_trq'].append(trq[5])
    return sensor_data
        
def get_camera_image_from_link(robot, link, pic_width, pic_height, fov,
                               nearval, farval):
    aspect = pic_width / pic_height
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearval,
                                                     farval)
    link_info = p.getLinkState(robot, link, 1, 1)  #Get head link info
    link_pos = link_info[0]  #Get link com pos wrt world
    link_ori = link_info[1]  #Get link com ori wrt world
    rot = p.getMatrixFromQuaternion(link_ori)
    rot = np.array(rot).reshape(3, 3)

    global_camera_x_unit = np.array([1, 0, 0])
    global_camera_z_unit = np.array([0, 0, 1])

    camera_eye_pos = link_pos + np.dot(rot, 0.1 * global_camera_x_unit)
    camera_target_pos = link_pos + np.dot(rot, 1.0 * global_camera_x_unit)
    camera_up_vector = np.dot(rot, global_camera_z_unit)
    view_matrix = p.computeViewMatrix(camera_eye_pos, camera_target_pos,
                                      camera_up_vector)  #SE3_camera_to_world
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        pic_width,  #image width
        pic_height,  #image height
        view_matrix,
        projection_matrix)
    return width, height, rgb_img, depth_img, seg_img, view_matrix, projection_matrix, camera_eye_pos


def make_video(video_dir, delete_jpgs=True):
    images = []
    for file in tqdm(sorted(os.listdir(video_dir)),
                     desc='converting jpgs to gif'):
        filename = video_dir + '/' + file
        im = cv2.imread(filename)
        im = im[:, :, [2, 1, 0]]  # << BGR to RGB
        images.append(im)
        if delete_jpgs:
            os.remove(filename)
    imageio.mimsave(video_dir + '/video.gif', images[:-1], duration=0.01)


def get_camera_image(cam_target_pos, cam_dist, cam_yaw, cam_pitch, cam_roll,
                     fov, render_width, render_height, nearval, farval):
    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=cam_target_pos,
        distance=cam_dist,
        yaw=cam_yaw,
        pitch=cam_pitch,
        roll=cam_roll,
        upAxisIndex=2)
    proj_matrix = p.computeProjectionMatrixFOV(fov=fov,
                                               aspect=float(render_width) /
                                               float(render_height),
                                               nearVal=nearval,
                                               farVal=farval)
    (_, _, px, _, _) = p.getCameraImage(width=render_width,
                                        height=render_height,
                                        renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                        viewMatrix=view_matrix,
                                        projectionMatrix=proj_matrix)
    rgb_array = np.array(px, dtype=np.uint8)
    rgb_array = np.reshape(np.array(px), (render_height, render_width, -1))
    rgb_array = rgb_array[:, :, :3]

    return rgb_array


def is_key_triggered(keys, key):
    print(key)    
    o = ord(key)
    print(o)
    if o in keys:
        print('key is triggered')        
        return keys[ord(key)] & p.KEY_WAS_TRIGGERED
    return False

def get_key_pressed():
    pressed_keys = []
    events = p.getKeyboardEvents()
    key_codes = events.keys()
    for key in key_codes:
        pressed_keys.append(key)
    return pressed_keys

def set_config(robot, joint_id, joint_pos):
    for k, v in joint_pos.items():
        p.resetJointState(robot, joint_id[k], v, 0.)
        
def get_joint_dict(joint_id, joint_pos):
    joint_value = OrderedDict()
    for k,v in joint_id.items():
        joint_value[k] = joint_pos[int(v)]
    return joint_value


def get_point_cloud_data(depth_buffer, view_matrix, projection_matrix, d_hor,
                         d_ver):
    view_matrix = np.asarray(view_matrix).reshape([4, 4], order='F')
    projection_matrix = np.asarray(projection_matrix).reshape([4, 4],
                                                              order='F')
    trans_world_to_pix = np.linalg.inv(
        np.matmul(projection_matrix, view_matrix))
    trans_camera_to_pix = np.linalg.inv(projection_matrix)
    img_height = (depth_buffer.shape)[0]
    img_width = (depth_buffer.shape)[1]

    wf_point_cloud_data = np.empty(
        [np.int(img_height / d_ver),
         np.int(img_width / d_hor), 3])
    cf_point_cloud_data = np.empty(
        [np.int(img_height / d_ver),
         np.int(img_width / d_hor), 3])

    for h in range(0, img_height, d_ver):
        for w in range(0, img_width, d_hor):
            x = (2 * w - img_width) / img_width
            y = (2 * h - img_height) / img_height
            z = 2 * depth_buffer[h, w] - 1
            pix_pos = np.asarray([x, y, z, 1])
            point_in_world = np.matmul(trans_world_to_pix, pix_pos)
            point_in_camera = np.matmul(trans_camera_to_pix, pix_pos)
            wf_point_cloud_data[np.int(h / d_ver),
                                np.int(w / d_hor), :] = (
                                    point_in_world /
                                    point_in_world[3])[:3]  #world frame

            cf_point_cloud_data[np.int(h / d_ver),
                                np.int(w / d_hor), :] = (
                                    point_in_world /
                                    point_in_world[3])[:3]  #camera frame

    return wf_point_cloud_data, cf_point_cloud_data
