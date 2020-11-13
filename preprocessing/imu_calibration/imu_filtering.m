
ld = load('/home/zhanibek/codes/phhi_data_analysis/data/obs_example_4.mat');

rf1 = ld.rft1;
tpos = ld.pose123.time_steps;
position = ld.pose123.position;
orientation = ld.pose123.orientation;

[tpos, position, orientation] = smooth_pose(tpos, position, orientation);

aruco_rotm = axang2rotm(orientation);
% grfA frame seen from NED frame
gNA = [-1  0  0; 
        0  1  0;
        0  0 -1];

%% Plot all sensor data

% figure(11)
% 
% FT sensor
% subplot(5,2,1)
% plot(ld.rft1.time_steps, ld.rft1.force)
% title(sprintf('%s Force', ld.rft1.frame_id))
% grid on
% 
% subplot(5,2,2)
% plot(ld.rft1.time_steps, ld.rft1.torque)
% title(sprintf('%s Torque', ld.rft1.frame_id))
% grid on
% 
% 
% subplot(5,2,3)
% plot(ld.rft2.time_steps, ld.rft2.force)
% title(sprintf('%s Force', ld.rft2.frame_id))
% grid on
% 
% subplot(5,2,4)
% plot(ld.rft2.time_steps, ld.rft2.torque)
% title(sprintf('%s Torque', ld.rft2.frame_id))
% grid on
% legend('x','y','z')
% 
% 
% Position
% subplot(5,2,5)
% plot(ld.pose123.time_steps, ld.pose123.position,'.')
% title(sprintf('%s Position', ld.pose123.frame_id))
% grid on
% 
% subplot(5,2,6)
% plot(ld.pose123.time_steps, ld.pose123.orientation,'.')
% title(sprintf('%s Orientation', ld.pose123.frame_id))
% grid on
% legend('x','y','z','w')
% 
% 
% 
% IMU
% subplot(5,2,7)
% plot(ld.imu.time_steps, ld.imu.accel)
% title(sprintf('%s Acceleration', ld.imu.frame_id))
% grid on
% 
% subplot(5,2,8)
% plot(ld.imu.time_steps, ld.imu.gyro)
% title(sprintf('%s Gyro', ld.imu.frame_id))
% grid on
% 
% subplot(5,2,9)
% plot(ld.imu.time_steps, ld.imu.mag)
% title(sprintf('%s Mag', ld.imu.frame_id))
% grid on
% legend('x','y','z','w')

%% Plot Position and IMU
figure(21)
% Position
subplot(3,2,1)
plot(tpos, position)
subtitle(sprintf('%s Position', ld.pose123.frame_id))
grid on

subplot(3,2,2)
plot(tpos, orientation)
subtitle(sprintf('%s Orientation', ld.pose123.frame_id))
grid on
legend('x','y','z','w')


% IMU
subplot(3,2,3)
plot(ld.imu.time_steps, ld.imu.accel)
subtitle(sprintf('%s Acceleration', ld.imu.frame_id))
grid on

subplot(3,2,4)
plot(ld.imu.time_steps, ld.imu.gyro)
subtitle(sprintf('%s Gyro', ld.imu.frame_id))
grid on

subplot(3,2,5)
plot(ld.imu.time_steps, ld.imu.mag)
subtitle(sprintf('%s Mag', ld.imu.frame_id))
grid on
legend('x','y','z')
sgtitle('Position and IMU')


%% IMU Filtering - Orientation

% 1 Gs, G = 0.0001 T
accel = ld.imu.accel;
gyro = ld.imu.gyro/180*pi;
mag = ld.imu.mag * 100 ; % convert from gauss to micro Tesla. Mag is pre calibrated
timu = ld.imu.time_steps;
fs = 1/mean(diff(timu));

% Rned2wall = [-0.0006   -1.0000    0.0062;
%               0.9998   -0.0005    0.0174;
%              -0.0174    0.0062    0.9998];
%          

% bring back from tray coordinates to sensor coordinates
% accel(:,2) = -accel(:,2);
% accel(:,3) = -accel(:,3);
% 
% gyro(:,2) = -gyro(:,2);
% gyro(:,3) = -gyro(:,3);
% 
% mag(:,2) = -mag(:,2);
% mag(:,3) = -mag(:,3);


expmfs = 0.4166 * 100;

filt = ahrsfilter('ReferenceFrame', 'NED','OrientationFormat','Rotation matrix');%, ... 
%                   'ExpectedMagneticFieldStrength',expmfs);%, ...
%                   'AccelerometerNoise', 0.01,...
%                   'MagnetometerNoise', 1);
filt.SampleRate = fs;
% Outputs the orientation of Tray wrt NED
[rotmats,angvel] = filt(accel, gyro, mag);


% Convert from NED to grfA frame
for i=1:size(rotmats,3)
    rotmats(:,:,i) = gAN*rotmats(:,:,i);
end

orient = rotm2axang(rotmats);
% orient = orient.*sign(orient(:,3));

% Plot Filtered Pose

figure(31)
% Position
subplot(2,2,1)
plot(tpos, position)
subtitle(sprintf('%s Position', 'Aruco'))
grid on

subplot(2,2,2)
plot(tpos, orientation)
subtitle(sprintf('%s Orientation', 'Aruco'))
grid on
legend('x','y','z','w')


subplot(2,2,3)
plot(timu, angvel)
subtitle(sprintf('%s Angular Velocty', 'KF'))
grid on

subplot(2,2,4)
plot(timu, orient)
subtitle(sprintf('%s Orientation', 'KF'))
grid on
legend('x','y','z','angle')
sgtitle('Aruco vs Kalman Filter')

