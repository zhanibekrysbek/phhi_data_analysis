function observations_processed = adjust_time(observations_processed)
%adjust_time Trim out observations at nominal start and end times.
%   T_start is defined as 0.5 seconds earlier from the instance when Fz>5 (rising edge).
%   Similarly T_end is defined 0.5 seconds after Fz<5 (falling edge).



for ind = progress(1:numel(observations_processed), 'Title', 'Adjusting Time')
    obs = observations_processed(ind);
    if strcmp(obs.obs_id, 'Zhanibek_Sanket_2')
        obs = shift_time(obs, .5, 3.5);
    elseif strcmp(obs.obs_id, 'Sanket_Vignesh_2_19')
        obs = shift_time(obs, 1., 1.);
    else
        % adjust by 0.5 secons on both sides 
        obs = shift_time(obs, .5, .5);
    end
    I1 = obs.rft1.force(:,3)>5;
    I2 = obs.rft2.force(:,3)>5;

    t0 = min(min(obs.rft1.time_steps(I1)), min(obs.rft2.time_steps(I2)))-0.5;
    tf = max(max(obs.rft1.time_steps(I1)), max(obs.rft2.time_steps(I2)))+0.5;
    offset0 = round(t0-0.005, 2);
    offset1 = round(obs.rft1.time_steps(end)-tf-0.05,2);
    
    observations_processed(ind) = shift_time(obs, offset0, offset1);
end
end



function obs = shift_time(obs, offset0, offset1)
    eps = 1e-5;
    t0 = round(offset0-0.005,2)-eps;
    
    tf = round(obs.rft1.time_steps(end)-offset1-0.005, 2)+eps;
    
    I = obs.rft1.time_steps >= t0 & obs.rft1.time_steps <= tf;

    obs.rft1.time_steps = obs.rft1.time_steps(I) - t0;
    obs.rft1.time_steps = obs.rft1.time_steps - obs.rft1.time_steps(1);
    obs.rft1.force = obs.rft1.force(I,:);
    obs.rft1.forceS = obs.rft1.forceS(I,:);
    obs.rft1.torque = obs.rft1.torque(I,:);
    obs.rft1.torqueS = obs.rft1.torqueS(I,:);

    obs.rft2.time_steps = obs.rft2.time_steps(I) - t0;
    obs.rft2.time_steps = obs.rft2.time_steps - obs.rft2.time_steps(1);
    obs.rft2.force = obs.rft2.force(I,:);
    obs.rft2.forceS = obs.rft2.forceS(I,:);
    obs.rft2.torque = obs.rft2.torque(I,:);
    obs.rft2.torqueS = obs.rft2.torqueS(I,:);

    obs.fsum.time_steps = obs.fsum.time_steps(I) - t0;
    obs.fsum.time_steps = obs.fsum.time_steps - obs.fsum.time_steps(1);
    obs.fsum.force = obs.fsum.force(I,:);
    obs.fsum.forceS = obs.fsum.forceS(I,:);

    obs.fstretch.time_steps = obs.fstretch.time_steps(I) - t0;
    obs.fstretch.time_steps = obs.fstretch.time_steps - obs.fstretch.time_steps(1);
    obs.fstretch.force = obs.fstretch.force(I,:);
    obs.fstretch.forceS = obs.fstretch.forceS(I,:);

    % Pose
    I = obs.pose123.time_steps >= t0 & obs.pose123.time_steps <= tf;

    obs.pose123.time_steps = obs.pose123.time_steps(I) - t0;
    obs.pose123.time_steps = obs.pose123.time_steps - obs.pose123.time_steps(1);
    obs.pose123.position = obs.pose123.position(I,:);
    obs.pose123.orientation = obs.pose123.orientation(I,:);
    obs.pose123.linvel = obs.pose123.linvel(I,:);
    obs.pose123.linvelB = obs.pose123.linvelB(I,:);
    obs.pose123.linacc = obs.pose123.linacc(I,:);
    obs.pose123.linaccB = obs.pose123.linaccB(I,:);

    % IMU
    obs.imu.time_steps = obs.imu.time_steps(I) - t0;
    obs.imu.time_steps = obs.imu.time_steps - obs.imu.time_steps(1);
    obs.imu.accel = obs.imu.accel(I,:);
    obs.imu.gyro = obs.imu.gyro(I,:);
    obs.imu.mag = obs.imu.mag(I,:);
end
