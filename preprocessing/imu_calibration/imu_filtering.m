
base_path = '../../data/preprocessed_v2_1';

[observations,tb] = load_data(base_path);
observations_processed = adjust_time(observations);

%%
obs = observations_processed(1);


aruco_rotm = axang2rotm(obs.pose123.orientation);
% grfA frame seen from NED frame
gNA = [-1  0  0; 
        0  1  0;
        0  0 -1];

%% Plot all sensor data

figure(11)

% FT sensor
subplot(5,2,1)
plot(obs.rft1.time_steps, obs.rft1.force)
title(sprintf('%s Force', obs.rft1.frame_id))
grid on

subplot(5,2,2)
plot(obs.rft1.time_steps, obs.rft1.torque)
title(sprintf('%s Torque', obs.rft1.frame_id))
grid on


subplot(5,2,3)
plot(obs.rft2.time_steps, obs.rft2.force)
title(sprintf('%s Force', obs.rft2.frame_id))
grid on

subplot(5,2,4)
plot(obs.rft2.time_steps, obs.rft2.torque)
title(sprintf('%s Torque', obs.rft2.frame_id))
grid on
legend('x','y','z')


% Position
subplot(5,2,5)
plot(obs.pose123.time_steps, obs.pose123.position)
title(sprintf('%s Position', obs.pose123.frame_id))
grid on

subplot(5,2,6)
plot(obs.pose123.time_steps, obs.pose123.orientation)
title(sprintf('%s Orientation', obs.pose123.frame_id))
grid on
legend('x','y','z','w')



% IMU
subplot(5,2,7)
plot(obs.imu.time_steps, obs.imu.accel)
title(sprintf('%s Acceleration', obs.imu.frame_id))
grid on

subplot(5,2,8)
plot(obs.imu.time_steps, obs.imu.gyro)
title(sprintf('%s Gyro', obs.imu.frame_id))
grid on

subplot(5,2,9)
plot(obs.imu.time_steps, obs.imu.mag)
title(sprintf('%s Mag', obs.imu.frame_id))
grid on
legend('x','y','z','w')

%% Plot Position and IMU
figure(21)
% Position
subplot(3,2,1)
plot(obs.pose123.time_steps, obs.pose123.position)
subtitle(sprintf('%s Position', obs.pose123.frame_id))
grid on

subplot(3,2,2)
plot(obs.pose123.time_steps, obs.pose123.orientation)
subtitle(sprintf('%s Orientation', obs.pose123.frame_id))
grid on
legend('x','y','z','w')


% IMU
subplot(3,2,3)
plot(obs.imu.time_steps, obs.imu.accel)
subtitle(sprintf('%s Acceleration', obs.imu.frame_id))
grid on

subplot(3,2,4)
plot(obs.imu.time_steps, obs.imu.gyro)
subtitle(sprintf('%s Gyro', obs.imu.frame_id))
grid on

subplot(3,2,5)
plot(obs.imu.time_steps, obs.imu.mag)
subtitle(sprintf('%s Mag', obs.imu.frame_id))
grid on
legend('x','y','z')
sgtitle('Position and IMU')


%% IMU Filtering - Orientation

% 1 Gs, G = 0.0001 T
accel = obs.imu.accel;
gyro = obs.imu.gyro/180*pi;
mag = obs.imu.mag * 100 ; % convert from gauss to micro Tesla. Mag is pre calibrated
timu = obs.imu.time_steps;
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
%                   'ExpectedMagneticFieobsStrength',expmfs);%, ...
%                   'AccelerometerNoise', 0.01,...
%                   'MagnetometerNoise', 1);
filt.SampleRate = fs;
% Outputs the orientation of Tray wrt NED
[rotmats,angvel] = filt(accel, gyro, mag);


% Convert from NED to grfA frame
for i=1:size(rotmats,3)
    rotmats(:,:,i) = gNA*rotmats(:,:,i);
end

orient = rotm2axang(rotmats);
% orient = orient.*sign(orient(:,3));

% Plot Filtered Pose

figure(31)
% Position
subplot(2,2,1)
plot(obs.pose123.time_steps, obs.pose123.position)
subtitle(sprintf('%s Position', 'Aruco'))
grid on

subplot(2,2,2)
plot(obs.pose123.time_steps, obs.pose123.orientation)
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


%% Double Integrate

vx = zeros(size(obs.imu.time_steps));
x = zeros(size(obs.imu.time_steps));
ramp = linspace(0.7,0.5,numel(obs.imu.time_steps));
acc = obs.imu.accelS(:,1);
for i = 2:numel(x)
vx(i) = trapz(obs.imu.time_steps(1:i), acc(1:i));
end

for i = 2:numel(x)
x(i) = trapz(obs.imu.time_steps(1:i), vx(1:i));
end

figure(1);
subplot(211)
plot(obs.imu.time_steps,vx);hold on;
plot(obs.pose123.time_steps, obs.pose123.position(:,1)); 
plot(obs.pose123.time_steps, x), hold off;

subplot(212)
plot(obs.imu.time_steps, acc)


