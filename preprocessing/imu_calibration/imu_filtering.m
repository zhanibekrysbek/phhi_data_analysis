
ld = load('/home/zhanibek/codes/phhi_data_analysis/data/obs_example.mat');

rf1 = ld.rft1;

%%

figure(1)

% FT sensor
subplot(5,2,1)
plot(ld.rft1.time_steps, ld.rft1.force)
title(sprintf('%s Force', ld.rft1.frame_id))
grid on

subplot(5,2,2)
plot(ld.rft1.time_steps, ld.rft1.torque)
title(sprintf('%s Torque', ld.rft1.frame_id))
grid on


subplot(5,2,3)
plot(ld.rft2.time_steps, ld.rft2.force)
title(sprintf('%s Force', ld.rft2.frame_id))
grid on

subplot(5,2,4)
plot(ld.rft2.time_steps, ld.rft2.torque)
title(sprintf('%s Torque', ld.rft2.frame_id))
grid on
legend('x','y','z')


% Position
subplot(5,2,5)
plot(ld.pose123.time_steps, ld.pose123.position,'.')
title(sprintf('%s Position', ld.pose123.frame_id))
grid on

subplot(5,2,6)
plot(ld.pose123.time_steps, ld.pose123.orientation,'.')
title(sprintf('%s Orientation', ld.pose123.frame_id))
grid on
legend('x','y','z','w')



% IMU
subplot(5,2,7)
plot(ld.imu.time_steps, ld.imu.accel)
title(sprintf('%s Acceleration', ld.imu.frame_id))
grid on

subplot(5,2,8)
plot(ld.imu.time_steps, ld.imu.gyro)
title(sprintf('%s Gyro', ld.imu.frame_id))
grid on

subplot(5,2,9)
plot(ld.imu.time_steps, ld.imu.mag)
title(sprintf('%s Mag', ld.imu.frame_id))
grid on
legend('x','y','z','w')

%% Plot Position and IMU
figure(2)
% Position
subplot(3,2,1)
plot(ld.pose123.time_steps, ld.pose123.position,'.')
title(sprintf('%s Position', ld.pose123.frame_id))
grid on

subplot(3,2,2)
plot(ld.pose123.time_steps, ld.pose123.orientation,'.')
title(sprintf('%s Orientation', ld.pose123.frame_id))
grid on
legend('x','y','z','w')



% IMU
subplot(3,2,3)
plot(ld.imu.time_steps, ld.imu.accel)
title(sprintf('%s Acceleration', ld.imu.frame_id))
grid on

subplot(3,2,4)
plot(ld.imu.time_steps, ld.imu.gyro)
title(sprintf('%s Gyro', ld.imu.frame_id))
grid on

subplot(3,2,5)
plot(ld.imu.time_steps, ld.imu.mag)
title(sprintf('%s Mag', ld.imu.frame_id))
grid on
legend('x','y','z')



%% IMU Filtering

% 1 Gs, G = 0.0001 T
accel = ld.imu.accel;
gyro = ld.imu.gyro/180*pi;
mag = ld.imu.mag*1e-4*1e6;
timu = ld.imu.time_steps;

filt = imufilter('ReferenceFrame','ENU');
filt.SampleRate = 77.3081;


[orient,angvel] = filt(accel, gyro);
orient = quat2axang(orient);
orient2 = orient.*sign(orient(:,3));


%% Plot Filtered Pose

figure(3)
% Position
subplot(2,2,1)
plot(ld.pose123.time_steps, ld.pose123.position,'.')
title(sprintf('%s Position', ld.pose123.frame_id))
grid on

subplot(2,2,2)
plot(ld.pose123.time_steps, ld.pose123.orientation,'.')
title(sprintf('%s Orientation', ld.pose123.frame_id))
grid on
legend('x','y','z','w')


subplot(2,2,3)
plot(timu, angvel)
title(sprintf('%s Angular Veloctiy', ld.imu.frame_id))
grid on

subplot(2,2,4)
plot(timu, orient)
title(sprintf('%s Orientation', ld.imu.frame_id))
grid on
legend('x','y','z','w')

