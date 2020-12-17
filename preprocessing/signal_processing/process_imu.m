function [obs] = process_imu(obs,tf)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    tnom = 0:1/100:tf; % nominal time that will be common for both forces
    
%     accel = [obs.imu.accel(1,:); obs.imu.accel; position(end,:)];
%     orientation = [orientation(1,:);orientation; orientation(end,:)];
    
    method = 'pchip';
    obs.imu.accel = interp1(obs.imu.time_steps,obs.imu.accel, tnom, method);
    obs.imu.gyro = interp1(obs.imu.time_steps,obs.imu.gyro, tnom, method);
    obs.imu.mag = interp1(obs.imu.time_steps,obs.imu.mag, tnom, method);
    obs.imu.time_steps = tnom;
    
end

