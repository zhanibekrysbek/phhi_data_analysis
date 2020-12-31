


ld = load('../../data/magnetic_north.mat');

% Convert to Tray Reference Frame
ld.accel(:,2) = -ld.accel(:,2);
ld.accel(:,3) = -ld.accel(:,3);

ld.gyro(:,2) = -ld.gyro(:,2);
ld.gyro(:,3) = -ld.gyro(:,3);

ld.mag(:,2) = -ld.mag(:,2);
ld.mag(:,3) = -ld.mag(:,3);

ld.t = ld.t - ld.t(1);
ld_orig = ld;
%% Band Pass Filter

tnom = ld.t(1):1/100:ld.t(end);

method = 'pchip';
ld.accel = interp1(ld.t,ld.accel, tnom, method);
ld.gyro = interp1(ld.t,ld.gyro, tnom, method);
ld.mag = interp1(ld.t,ld.mag, tnom, method);
ld.t = tnom;


Fs=100;
lcutoff = 0.001;
hcutoff = 12.5;

for ax = 1:3
    ld.accel(:,ax) = bandpass_fft(ld.accel(:,ax), Fs, lcutoff, hcutoff);
    ld.mag(:,ax) = bandpass_fft(ld.mag(:,ax), Fs, lcutoff, hcutoff);
    ld.gyro(:,ax) = bandpass_fft(ld.gyro(:,ax), Fs, lcutoff, hcutoff);
end

%%

% 1 Gs, G = 0.0001 T
accel = ld.accel;
gyro = ld.gyro/180*pi;
gyro_mean = mean(gyro);
gyro = gyro - gyro_mean;
mag = ld.mag; % convert from gauss to micro Tesla
t = ld.t;
t = t - t(1);
fs = 1/mean(diff(t));

expmfs = 0.4167 * 100;
mag = (mag - b)*A * 100; % Calibrate Mag

% gyro = lowpass(gyro, 5, fs);
gNA = [-1  0  0; 
        0  1  0;
        0  0 -1];

% gyro = gyro - mean(gyro); % Calibrate Gyro

mag_vec = [0 0 0; mean(mag)];

%% Plot IMU values

figure(2)

subplot(3,2,1)
plot(ld_orig.t,ld_orig.mag)
subtitle('mag')
grid on
sgtitle('IMU')
ylabel('gauss')

subplot(3,2,3)
plot(ld_orig.t,ld_orig.accel)
subtitle('accel')
grid on
ylabel('m/s^2')

subplot(3,2,5)
plot(ld_orig.t,ld_orig.gyro)
subtitle('gyro')
ylabel('deg/s')
grid on
legend('x','y','z')
xlabel('time,sec')


subplot(3,2,2)
plot(t,mag)
subtitle('mag')
grid on
ylabel('\muT')

subplot(3,2,4)
plot(t,accel)
subtitle('accel')
grid on
ylabel('m/s^2')

subplot(3,2,6)
plot(t,gyro)
subtitle('gyro')
ylabel('rad/s')
grid on
legend('x','y','z')
xlabel('time,sec')

%% Mag 3D plot

figure(3)
scatter3(mag(:,1), mag(:,2), mag(:,3))
hold on;
plot3(mag_vec(:,1), mag_vec(:,2), mag_vec(:,3), 'o', 'MarkerFaceColor', 'red')
plot3(mag_vec(:,1), mag_vec(:,2), mag_vec(:,3), 'LineWidth',2)
hold off
xlim([-50 50])
ylim([-50 50])
zlim([-50 50])
xlabel('x'); ylabel('y'); zlabel('z')

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
% filt.SampleRate = 77.3081;
filt.SampleRate = 100;

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

