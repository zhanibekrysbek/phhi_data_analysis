#!/usr/bin/env python


import numpy as np
import sys
sys.path.insert(1,'/home/zhanibek/.local/lib/python2.7/site-packages/cv2')
sys.path.insert(1,'/home/zhanibek/catkin_ws/src/smart_tray/scripts/')
import cv2
import matplotlib.pyplot as plt
import os
from tqdm import tqdm
import pickle
import pandas as pd

from scipy import io as sio

import phri
from phri.utils import *


from math import ceil

from scipy import linalg

from sklearn import linear_model, datasets


meta_data_paths = [
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/koh_sanket/trial_0/koh_sanket_trial_0_2020-10-11-20-48-27_meta_data_v2.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/koh_zhanibek/trial_0/koh_zhanibek_trial_0_2020-10-11-21-19-07_meta_data_v2.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/sanket_vignesh/trial_0/trial_0_2020-10-12-14-47-43_meta_data_v2.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/sanket_vignesh/trial_1/trial_1_2020-10-12-14-51-53_meta_data_v2.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/zhanibek_sanket/trial_0/trial_0_2020-10-12-15-02-32_meta_data_v2.pkl',
    '/home/zhanibek/catkin_ws/src/smart_tray/data/rosbag/zhanibek_vignesh/trial_0/trial_0_2020-10-12-14-57-11_meta_data_v2.pkl'
]

ann_paths = ['/home/zhanibek/codes/phhi_data_analysis/data/annotation/annotation - KOH_Sanket.csv',
             '/home/zhanibek/codes/phhi_data_analysis/data/annotation/annotation - KOH_Zhanibek.csv',
             '/home/zhanibek/codes/phhi_data_analysis/data/annotation/annotation - Sanket_Vignesh_1.csv',
             '/home/zhanibek/codes/phhi_data_analysis/data/annotation/annotation - Sanket_Vignesh_2.csv',
             '/home/zhanibek/codes/phhi_data_analysis/data/annotation/annotation - Zhanibek_Sanket.csv',
             '/home/zhanibek/codes/phhi_data_analysis/data/annotation/annotation - Zhanibek_Vignesh.csv']

imu_delay_const = 52345.383253
imu_delays = np.array([0, 0, 0, 0, 0, 0])
imu_delays[[2, 3, 4, 5]] = imu_delay_const


# Transformation Constants
# gAS - A to grf
grfA = np.array(
    [[-1, 0, 0, 1.42],
     [0, -1, 0, 0.12],
     [0,  0, 1, 0],
     [0,  0, 0, 1]])
# gBS - B to grf
grfB = np.array(
    [[1, 0, 0, 1.27],
     [0, 1, 0, -0.12],
     [0, 0, 1, 0],
     [0, 0, 0, 1]])


# Calibrate Magnetometer to Soft-Hard Iron Effects
def calibrate_mag(mag):

    A = np.array(
        [[0.9249, 0.0957, -0.0063],
         [0.0957, 1.0708, -0.0565],
         [-0.0063, -0.0565, 1.0221]])

    b = np.array([0.0377, 0.1454, -0.0653])

    expmfs = 0.4166

    mag = np.matmul(mag-b, A)

    return mag


def read_rft(tt0, ttf, temp_df):

    cut = temp_df.time_stamp.apply(lambda x: x >= tt0 and x <= ttf)
    temp = temp_df[cut]

    force = dict(temp)['force'].to_numpy()
    force = np.array([[f[0], f[1], f[2]] for f in force])

    torque = dict(temp)['torque'].to_numpy()
    torque = np.array([[f[0], f[1], f[2]] for f in torque])

    tsteps = dict(temp)['time_stamp'].to_numpy()
    tsteps -= tt0
    frame_id = temp_df.frame_id.iloc[0]

    new_force = np.zeros(force.shape)
    new_torque = new_force.copy()
    # Convert to Tray Reference Frame
    # Sensor 1
    if frame_id == 'C00300119':
        new_force[:, 0] = force[:, 2]
        new_force[:, 1] = force[:, 0]
        new_force[:, 2] = force[:, 1]

        new_torque[:, 0] = torque[:, 2]
        new_torque[:, 1] = torque[:, 0]
        new_torque[:, 2] = torque[:, 1]
    # Sensor 2
    elif frame_id == 'C00300122':
        new_force[:, 0] = -force[:, 2]
        new_force[:, 1] = -force[:, 0]
        new_force[:, 2] = force[:, 1]

        new_torque[:, 0] = -torque[:, 2]
        new_torque[:, 1] = -torque[:, 0]
        new_torque[:, 2] = torque[:, 1]

    res = {'time_steps': tsteps, 'force': new_force,
           'torque': new_torque, 'frame_id': frame_id}

    return res


def read_imu(tt0, ttf, temp_df):

    cut = temp_df.time_stamp.apply(lambda x: x >= tt0 and x <= ttf)
    temp = temp_df[cut]

    accel = dict(temp)['accel'].to_numpy()
    accel = np.array([[f[0], f[1], f[2]] for f in accel])

    gyro = dict(temp)['gyro'].to_numpy()
    gyro = np.array([[f[0], f[1], f[2]] for f in gyro])

    mag = dict(temp)['mag'].to_numpy()
    mag = np.array([[f[0], f[1], f[2]] for f in mag])
    tsteps = dict(temp)['time_stamp'].to_numpy()
    tsteps -= tt0

    mag = calibrate_mag(mag)

    # Transform from Sensor Coordinate frame to Tray Coordinates
    new_accel = np.zeros(accel.shape)
    new_gyro = new_accel.copy()
    new_mag = new_gyro.copy()

    new_accel[:, 0] = accel[:, 0]
    new_accel[:, 1] = -accel[:, 1]
    new_accel[:, 2] = -accel[:, 2]

    new_mag[:, 0] = mag[:, 0]
    new_mag[:, 1] = -mag[:, 1]
    new_mag[:, 2] = -mag[:, 2]

    new_gyro[:, 0] = gyro[:, 0]
    new_gyro[:, 1] = -gyro[:, 1]
    new_gyro[:, 2] = -gyro[:, 2]

    res = {'time_steps': tsteps, 'accel': new_accel, 'gyro': new_gyro,
           'mag': new_mag, 'frame_id': temp.frame_id.iloc[0]}

    return res


def read_pose(tt0, ttf, pose_df):

    cut = pose_df.time_stamp.apply(lambda x: x >= tt0 and x <= ttf)
    temp = pose_df[cut]

    position = dict(temp)['position'].to_numpy()
    position = np.array([[f[0], f[1], f[2]] if not np.isnan(
        f).any() else [np.nan, np.nan, np.nan] for f in position])

    quat = dict(temp)['quaternion'].to_numpy()
    quat = np.array([[f[0], f[1], f[2], f[3]] if not np.isnan(
        f).any() else [np.nan, np.nan, np.nan, np.nan] for f in quat])

    tsteps = dict(temp)['time_stamp'].to_numpy()
    tsteps -= tt0

    res = {'time_steps': tsteps, 'position': position,
           'quaternion': quat, 'frame_id': pose_df.frame_id.iloc[0]}

    return res


def merge_offline_pos(pos, pos_off, txt='_merged'):
    if len(pos['position']) == 0 and np.isnan(pos_off['position']).all():
        return {'time_steps': [], 'position': [], 'quaternion': [], 'frame_id': pos['frame_id']+txt}
    elif len(pos['position']) == 0:
        res = {'time_steps': pos_off['time_steps'],
               'position': pos_off['position'],
               'quaternion': pos_off['quaternion'],
               'frame_id': pos_off['frame_id']+txt}
        return res
    elif len(pos_off['position']) == 0:
        res = {'time_steps': pos['time_steps'],
               'position': pos['position'],
               'quaternion': pos['quaternion'],
               'frame_id': pos['frame_id']+txt}
        return res

    res = {}
    tt = np.concatenate((pos['time_steps'], pos_off['time_steps']))
    pp = np.concatenate((pos['position'], pos_off['position']))
    qq = np.concatenate((pos['quaternion'], pos_off['quaternion']))

    # drop duplicate values
    _, indx = np.unique(tt, return_index=True)

    tt = tt[indx]
    pp = pp[indx]
    qq = qq[indx]
    # sort once more timewise
    sind = np.argsort(tt)
    tt = tt[sind]
    pp = pp[sind]
    qq = qq[sind]

    res = {'time_steps': tt, 'position': pp,
           'quaternion': qq, 'frame_id': pos['frame_id']+txt}
    return res


def filter_outliers(pose123):
    orientation = pose123['orientation']
    position = pose123['position']
    t = pose123['time_steps']

    m_pos = np.array([2.5, 2.5, 2.5])
    m_orient = np.array([2.5, 2.5, 2.5, 3.5])

    fence_orient = np.quantile(orientation, [0.25, 0.75], axis=0)
    iqr = fence_orient[1]-fence_orient[0]
    fence_orient[0] -= m_orient*iqr
    fence_orient[1] += m_orient*iqr

    fence_pos = np.quantile(position, [0.25, 0.75], axis=0)
    iqr = fence_pos[1]-fence_pos[0]
    fence_pos[0] -= m_pos*iqr
    fence_pos[1] += m_pos*iqr

    outlier_mask = ((orientation > fence_orient[1]) +
                    (orientation < fence_orient[0])).any(axis=1)
    outlier_mask += ((position > fence_pos[1]) +
                     (position < fence_pos[0])).any(axis=1)
    inlier_mask = ~outlier_mask

    pose123['time_steps'] = pose123['time_steps'][inlier_mask]
    pose123['position'] = pose123['position'][inlier_mask]
    pose123['orientation'] = pose123['orientation'][inlier_mask]

    return pose123


def enforce_angle_range(pos123):

    pos123['orientation'][:, 3] = np.unwrap(pos123['orientation'][:, 3])

    return pos123


def transform2grfAB(pos123, traj_type):

    # Drop nans
    position = pos123['position'].copy()
    quaternion = pos123['quaternion'].copy()
    t = pos123['time_steps'].copy()

    # Drop old values
    pos123.pop('position', None)
    pos123.pop('time_steps', None)
    pos123.pop('quaternion', None)

    idx = ~np.isnan(position[:, 0])

    position = position[idx, :]
    quaternion = quaternion[idx, :]
    t = t[idx]

    orientation = np.zeros(quaternion.shape)
    position_new = np.zeros(position.shape)
    for ind in range(t.shape[0]):
        pos = position[ind, :]
        quat = quaternion[ind, :]

        gt = g_from_pose({'position': pos, 'orientation': quat})
        # fix left handed coordinate reference frame bug from aruco
        R = gt[:3, :3]
        if np.sign(np.linalg.det(R)) == -1:
            gt[:, 1] = -gt[:, 1]
            print('Left handed Frame is found!')

        if 'AB' in traj_type:
            g_new = np.matmul(grfA, gt)

        elif 'BA' in traj_type:
            g_new = np.matmul(grfB, gt)

        pose_new = pose_from_g(g_new, rotation='axis-angle', zup=True)
        position_new[ind, :] = pose_new['position']
        orientation[ind, :] = pose_new['orientation']

    pos123['position'] = position_new
    pos123['orientation'] = orientation
    pos123['time_steps'] = t

    if traj_type == 'BA1':
        traj_type = 'AB2'
    elif traj_type == 'BA2':
        traj_type = 'AB1'

    return pos123, traj_type




def save_observations(observations):
    base_path = '/home/zhanibek/codes/phhi_data_analysis/data/preprocessed_v1_2/'
    for obs_num in range(len(observations)):
        fname = observations[obs_num]['obs_id']+'.mat'
        sio.savemat(os.path.join(base_path,fname), observations[obs_num])

    return




def main():

    for ipath in tqdm(range(len(meta_data_paths))):
        # ipath = 0

        meta_data_path = meta_data_paths[ipath]
        ann_path = ann_paths[ipath]

        trial_pair = os.path.basename(ann_path).split('- ')[-1].split('.')[0]
        imu_delay = imu_delays[ipath]

        # Load the data and annotations
        print 'Loading: ', meta_data_path
        meta_data = pickle.load(open(meta_data_path))

        ann_df0 = pd.read_csv(ann_path)
        ann_df0.columns = ['obs_num', 'trajectory_type', 'motion_type', 'random', 't0_cam_1_seq',
                        't0_sec', 'tf_cam_1_seq', 'tf_sec', 'duration', 't0_precise', 'tf_precise',
                           'duration_precise', 'negotiation', 'tdec_sec', 'outcome', 'initialOrient', 'handle_1', 'handle_2']

        time_offset = float(ann_df0['t0_sec'][0])*1000

        ann_df = ann_df0[5:]
        

        # Cut the observations
        observations = []
        total_obs = ann_df.shape[0]
        rft1_df = meta_data['/RFT_FORCE']
        rft2_df = meta_data['/RFT_FORCE_2']
        imu_df = meta_data['/imu_data'].copy()
        imu_df.time_stamp += imu_delay

        pose1_df = meta_data['/cam1_tray_grf']
        pose1_off_df = meta_data['camera_1_grf_offline']

        pose2_df = meta_data['/cam2_tray_grf']
        pose2_off_df = meta_data['camera_2_grf_offline']

        pose3_df = meta_data['/cam3_tray_grf']
        pose3_off_df = meta_data['camera_3_grf_offline']

        for obs_num in tqdm(range(1, total_obs+1)):

            tt0 = float(ann_df.t0_precise[ann_df.obs_num == obs_num])+time_offset - 1.5
            ttf = float(ann_df.tf_precise[ann_df.obs_num == obs_num])+time_offset + 1.5
            # time when subjects decided the path
            tdec_sec = float(ann_df.tdec_sec[ann_df.obs_num == obs_num].iloc[0]) + time_offset - tt0

            rft1 = read_rft(tt0, ttf, rft1_df)
            rft2 = read_rft(tt0, ttf, rft2_df)

            imu = read_imu(tt0, ttf, imu_df)

            # in total we have pose data from 6 sources (cam_1, cam_1_offline, 2, 3)
            # Merge them into one
            pose1_grf = read_pose(tt0, ttf, pose1_df)
            pose1_off_grf = read_pose(tt0, ttf, pose1_off_df)
            pose1_merged = merge_offline_pos(pose1_grf, pose1_off_grf)

            pose2_grf = read_pose(tt0, ttf, pose2_df)
            pose2_off_grf = read_pose(tt0, ttf, pose2_off_df)
            pose2_merged = merge_offline_pos(pose2_grf, pose2_off_grf)

            pose3_grf = read_pose(tt0, ttf, pose3_df)
            pose3_off_grf = read_pose(tt0, ttf, pose3_off_df)
            pose3_merged = merge_offline_pos(pose3_grf, pose3_off_grf)

            pos123 = merge_offline_pos(pose1_merged, pose2_merged, txt='123')
            pos123 = merge_offline_pos(pos123, pose3_merged, txt='123')

            # Get the label for observations
            traj_type = ann_df.trajectory_type[ann_df.obs_num == obs_num].iloc[0]
            motion_type = ann_df.motion_type[ann_df.obs_num == obs_num].iloc[0]
            
            outcome = ann_df.outcome[ann_df.obs_num == obs_num].iloc[0]
            initialOrient = ann_df.initialOrient[ann_df.obs_num == obs_num].iloc[0]
            handle_1 = ann_df.handle_1[ann_df.obs_num == obs_num].iloc[0]
            handle_2 = ann_df.handle_2[ann_df.obs_num == obs_num].iloc[0]

            pos123, traj_type = transform2grfAB(pos123, traj_type)
            pos123 = enforce_angle_range(pos123)

        #     pos123 = filter_outliers(pos123)

            # pos123 = enforce_angle_range(pos123)

            obs = {'rft1': rft1, 'rft2': rft2, 'imu': imu, 'pose123': pos123,
                'traj_type': traj_type, 'motion_type': motion_type, 'outcome':outcome, 
                   'tdec_sec': tdec_sec, 'initialOrient': initialOrient, 'handle_1': handle_1,
                'handle_2':handle_2, 'obs_id': trial_pair+'_'+str(obs_num)}

            observations.append(obs)

        save_observations(observations)
    return

if __name__ == '__main__':
    main()




