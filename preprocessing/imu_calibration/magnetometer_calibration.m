
ld = load('/home/zhanibek/codes/phhi_data_analysis/data/magnetometer_calib.mat');

t = ld.t;
t = t-t(1);
mag = ld.mag;
accel = ld.accel;
gyro = ld.gyro;

figure(1)
subplot(1,2,1)

scatter3(mag(:,1),mag(:,2),mag(:,3))
grid on


[A,b,expmfs] = magcal(mag);
mag_cal = (mag-b)*A;
subplot(1,2,2)
scatter3(mag_cal(:,1),mag_cal(:,2),mag_cal(:,3))
grid on


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




