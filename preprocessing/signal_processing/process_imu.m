function [obs] = process_imu(obs,tf)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    tnom = 0:1/100:tf; % nominal time that will be common for both forces
    
%     accel = [obs.imu.accel(1,:); obs.imu.accel; position(end,:)];
%     orientation = [orientation(1,:);orientation; orientation(end,:)];
    
    obs = calibrate_mag(obs);
    
    % Temporal alignment to nominal time. This increases the sampling frequency
    % from 76.3Hz to 100Hz
    method = 'pchip';
    obs.imu.accel = interp1(obs.imu.time_steps,obs.imu.accel, tnom, method);
    obs.imu.gyro = interp1(obs.imu.time_steps,obs.imu.gyro, tnom, method);
    obs.imu.mag = interp1(obs.imu.time_steps,obs.imu.mag, tnom, method);
    obs.imu.time_steps = tnom;
    
    % Apply Low Pass filter
    obs = lowpass_imu(obs);
    
    % Transform to Spatial frame
    obs = to_spatial_frame(obs);
    
end


function obs = lowpass_imu(obs)

Fs=100;
lcutoff = 0.05;
hcutoff = 12.5;

for ax = 1:3
    obs.imu.accel(:,ax) = bandpass_fft(obs.imu.accel(:,ax), Fs, lcutoff, hcutoff);
    obs.imu.mag(:,ax) = bandpass_fft(obs.imu.mag(:,ax), Fs, lcutoff, hcutoff);
    obs.imu.gyro(:,ax) = bandpass_fft(obs.imu.gyro(:,ax), Fs, lcutoff, hcutoff);
end

end


function obs = to_spatial_frame(obs)


    % Get Spatial IMU
    rotm = axang2rotm(obs.pose123.orientation);
    
    accelS = zeros(size(obs.imu.accel));
    magS = zeros(size(obs.imu.mag));
    gyroS = zeros(size(obs.imu.gyro));
    
    for i=1:length(accelS)
        accelS(i,:) = rotm(:,:,i)*obs.imu.accel(i,:)';
        magS(i,:) = rotm(:,:,i)*obs.imu.mag(i,:)';
        gyroS(i,:) = rotm(:,:,i)*obs.imu.gyro(i,:)';
    end
    
    obs.imu.accelS = accelS;
    obs.imu.magS = magS;
    obs.imu.gyroS = gyroS;

end


function obs = calibrate_mag(obs)
A = [
    0.9249    0.0957   -0.0063;
    0.0957    1.0708   -0.0565;
   -0.0063   -0.0565    1.0221];
b = [0.0377    0.1454   -0.0653];
% expmfs = 0.4166;

% Calibrate and conver from gauss to micro Tesla
obs.imu.mag = (obs.imu.mag - b)*A*100;

end


