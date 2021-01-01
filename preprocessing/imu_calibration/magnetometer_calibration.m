
ld = load('../../data/magnetometer_calib.mat');

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

%% Plot Comparison

figure(1);
subplot(321);
plot(ld.t,ld.accel(:,1)); grid on;
subplot(323);
plot(ld.t,ld.accel(:,2)); grid on;
subplot(325);
plot(ld.t,ld.accel(:,3)); grid on;


subplot(322);
plot(ld_orig.t,ld_orig.accel(:,1)); grid on;
subplot(324);
plot(ld_orig.t,ld_orig.accel(:,2)); grid on;
subplot(326);
plot(ld_orig.t,ld_orig.accel(:,3)); grid on;


%% 3D plot

t = ld.t;
t = t-t(1);
mag = ld.mag;
accel = ld.accel;
gyro = ld.gyro;

figure(1)
subplot(1,2,1)

scatter3(mag(:,1),mag(:,2),mag(:,3))
grid on;
axis equal;


[A,b,expmfs] = magcal(mag);
mag_cal = (mag-b)*A;
subplot(1,2,2)
scatter3(mag_cal(:,1),mag_cal(:,2),mag_cal(:,3))
grid on;
axis equal;


%% Plot raw calibration data

figure(2)
subplot(3,1,1)
plot(t,mag)
title('mag')
grid on

subplot(3,1,2)
plot(t,accel)
title('accel')
grid on

subplot(3,1,3)
plot(t,gyro)
title('gyro')
grid on
legend('x','y','z')


% fprintf('\nA= %f\n',A);

A,b,expmfs




