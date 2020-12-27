


ld = load('../../data/magnetic_north.mat');

% close; clc;

% 1 Gs, G = 0.0001 T
accel = ld.accel;
gyro = ld.gyro/180*pi;
mag = ld.mag; % convert from gauss to micro Tesla
t = ld.t;
t = t - t(1);
fs = 1/mean(diff(t));

expmfs = 0.4166 * 100;
mag = (mag - b)*A * 100; % Calibrate Mag

gyro = lowpass(gyro, 5, fs);
gNA = [-1  0  0; 
        0  1  0;
        0  0 -1];

% gyro = gyro - mean(gyro); % Calibrate Gyro

% Convert to Tray Reference Frame
accel(:,2) = -accel(:,2);
accel(:,3) = -accel(:,3);

gyro(:,2) = -gyro(:,2);
gyro(:,3) = -gyro(:,3);

mag(:,2) = -mag(:,2);
mag(:,3) = -mag(:,3);

mag_vec = [0 0 0; mean(mag)];

%% Plot IMU values

figure(2)
subplot(3,1,1)
plot(t,mag)
subtitle('mag')
grid on
sgtitle('IMU')
ylabel('\muT')

subplot(3,1,2)
plot(t,accel)
subtitle('accel')
grid on
ylabel('m/s^2')

subplot(3,1,3)
plot(t,gyro)
subtitle('gyro')
ylabel('rad/s')
grid on
legend('x','y','z')
xlabel('time,sec')

%% Mag 3D plot

% figure(3)
% scatter3(mag(:,1), mag(:,2), mag(:,3))
% hold on;
% plot3(mag_vec(:,1), mag_vec(:,2), mag_vec(:,3), 'o', 'MarkerFaceColor', 'red')
% plot3(mag_vec(:,1), mag_vec(:,2), mag_vec(:,3), 'LineWidth',2)
% hold off
% xlim([-50 50])
% ylim([-50 50])
% zlim([-50 50])
% xlabel('x'); ylabel('y'); zlabel('z')

%% Mag vector 3D plot

figure(4)
plot3(mag_vec(:,1), mag_vec(:,2), mag_vec(:,3), 'LineWidth',2)
hold on;
plot3(mag_vec(:,1), mag_vec(:,2), mag_vec(:,3), 'o', 'MarkerFaceColor', 'red')
hold off;
title('Magnetic North direction')

grid on
xlim([-50 50])
ylim([-50 50])
zlim([-50 50])
xlabel('x'); ylabel('y'); zlabel('z')

%% NED to Lab Coordinate System

filt = ahrsfilter('OrientationFormat','Rotation matrix', 'ReferenceFrame','NED');
%                   'ReferenceFrame', 'NED', ... 
%                   'AccelerometerNoise', 0.01,...
%                   'MagnetometerNoise', 1);
              %                   'ExpectedMagneticFieldStrength',expmfs, ...
filt.SampleRate = 77.3081;

[rotmats,angvel] = filt(accel, gyro, mag);


% Convert NED to grfA frame
for i=1:size(rotmats,3)
    rotmats(:,:,i) = gNA'*rotmats(:,:,i);
end
orient = rotm2axang(rotmats);
% orient(:,4) = orient(:,4)/pi*180;
orient = orient.*sign(orient(:,3));

Rned2wall = rotmats(:,:,end);%*rotz(pi/2);

% Plot Filtered Pose
figure(5)
subplot(2,1,1)
plot(t, angvel)
subtitle(sprintf('%s Angular Veloctiy', 'Magnetic north'))
grid on

subplot(2,1,2)
plot(t, orient)
subtitle(sprintf('%s Orientation', 'Magnetic north'))
grid on
legend('x','y','z','angle')
sgtitle('Current pose wrt NED')

