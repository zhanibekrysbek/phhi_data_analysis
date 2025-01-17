#!/usr/local/bin/python3.7

import numpy as np
import sys
import imageio
print( sys.path)
# sys.path.insert(1,'/home/zhanibek/.local/lib/python2.7/site-packages/cv2')
sys.path.insert(1,'/home/zhanibek/catkin_ws/src/smart_tray/scripts/')
i = 0
while i < len(sys.path):
    if 'python2.7' in sys.path[i]:
        sys.path.pop(i)
        continue;
    i+=1


import cv2 
import matplotlib.pyplot as plt
import os
from tqdm import tqdm
from PIL import Image
from pygifsicle import optimize, gifsicle


import pickle
import pandas as pd
# from phri.utils import *



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
imu_delays = np.array([0,0,0,0,0,0])
imu_delays[[2,3,4,5]] = imu_delay_const


def read_image(fname):
    im = cv2.imread(fname)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    return im


def create_gif(tt0, ttf, temp_df, obs):
    
    base_path = '/home/zhanibek/codes/phhi_data_analysis/data/gifs'
    
    if 'angetube' in temp_df.frame_id[0]:
        cam_id = 'camera_1'
    elif 'logitech_t1' in temp_df.frame_id[0]:
        cam_id = 'camera_2'
    elif 'logitech_t2' in temp_df.frame_id[0]:
        cam_id = 'camera_3'
    
    cut = temp_df.time_stamp.apply(lambda x: x>=tt0 and x<=ttf)
    temp = temp_df[cut]
    
    gifname = obs['obs_id'] + '_' + obs['traj_type'] + '_' + obs['motion_type'] + '.gif'
    gifpath = os.path.join(base_path, cam_id, gifname)

    N = temp.shape[0];
    images = []
    for ind in range(N):

        impath = temp.image_path.iloc[ind]

        imname_aruco = os.path.basename(impath).split('.')[0]+'_aruco_offline.jpg'
        impath_aruco = os.path.join(os.path.split(impath)[0]+'_aruco_offline',imname_aruco) 
        
        if os.path.exists(impath_aruco):
            im = Image.open(impath_aruco)
#             if cam_id not in 'camera_3':
            # (width, height) = (im.width // 4, im.height // 4)
#             else:
#                 (width, height) = (im.width, im.height)
                
            # im_resized = im.resize((width, height), resample = Image.LANCZOS)
            images.append(im)
    
    images[0].save(gifpath, format='GIF', append_images=images[1:],
                    save_all=True, duration=100/3, loop=0, optimize=True);
    # optimize(gifpath)
    gifsicle(
        sources=[gifpath],  # or a single_file.gif
        # or just omit it and will use the first source provided.
        destination=gifpath,
        optimize=True,  # Whetever to add the optimize flag of not
        colors=128,  # Number of colors t use
        options=["--verbose"]  # Options to use.
    )
        


def main():
    
    for ipath in range(1): #len(meta_data_paths)):
    # ipath = 0

        meta_data_path = meta_data_paths[ipath]
        # ft_data_path = ft_data_paths[ipath]
        ann_path = ann_paths[ipath]
        trial_pair = os.path.basename(ann_path).split('- ')[-1].split('.')[0]
        imu_delay = imu_delays[ipath]


        meta_data = pickle.load(open(meta_data_path,'rb'), encoding='latin1')

        ann_df0 = pd.read_csv(ann_path)
        ann_df0.columns = ['obs_num', 'trajectory_type', 'motion_type', 'negotiation', 't0_cam_1_seq',
                        't0_sec', 'tf_cam_1_seq', 'tf_sec', 'duration', 't0_precise','tf_precise',
                        'duration_precise']

        time_offset = float(ann_df0['t0_sec'][0])*1000
        time_offset

        ann_df = ann_df0[5:]
        ann_df


        total_obs = ann_df.shape[0]
        observations = []

        camera_1 = meta_data['camera_1']
        camera_2 = meta_data['camera_2']
        camera_3 = meta_data['camera_3']

        for obs_num in tqdm(range(1,total_obs+1)):

            tt0 = float(ann_df.t0_precise[ann_df.obs_num==obs_num])+time_offset - 1.0
            ttf = float(ann_df.tf_precise[ann_df.obs_num==obs_num])+time_offset + 1.0
            
            traj_type = ann_df.trajectory_type[ann_df.obs_num==obs_num].iloc[0]
            motion_type = ann_df.motion_type[ann_df.obs_num==obs_num].iloc[0]
            
            obs = {'traj_type':traj_type, 'motion_type': motion_type, 'obs_id':trial_pair+'_'+str(obs_num)}    
            
            create_gif(tt0, ttf, camera_1, obs)
            create_gif(tt0, ttf, camera_2, obs)
            create_gif(tt0, ttf, camera_3, obs)
            
            observations.append(obs)
        



if __name__ == '__main__':
    main();


        


