function [obs] = process_imu(obs,tf)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    tnom = 0:1/100:tf; % nominal time that will be common for both forces
    
%     accel = [obs.imu.accel(1,:); obs.imu.accel; position(end,:)];
%     orientation = [orientation(1,:);orientation; orientation(end,:)];
    
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
cutoff = 12.5;

for ax = 1:3
    obs.imu.accel(:,ax) = lowpass_fft(obs.imu.accel(:,ax), Fs, cutoff);
    obs.imu.mag(:,ax) = lowpass_fft(obs.imu.mag(:,ax), Fs, cutoff);
    obs.imu.gyro(:,ax) = lowpass_fft(obs.imu.gyro(:,ax), Fs, cutoff);
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




