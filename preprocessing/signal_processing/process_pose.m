function [obs] = process_pose(obs, tf)
%process_pose Processes pose data from ARUCO
%   1. Outlier removal using quantile ranges
%   2. Interpolation at nominal 100Hz
%   3. Smoothing using Robust Lowess method
%   4. Computes Linear velocity from smoothed position data

    obs = outlier_removal(obs);
    position = obs.pose123.position;
    orientation = obs.pose123.orientation;
    tpos = obs.pose123.time_steps;
    
    tpos = [0 tpos tf];
    position = [position(1,:); position; position(end,:)];
    orientation = [orientation(1,:);orientation; orientation(end,:)];

    % Interpolation
    fs = 100;
    tnom = 0:1/fs:tpos(end);

    method = 'pchip';
    pos_interp = interp1(tpos,position,tnom, method);
    orient_interp = interp1(tpos,orientation,tnom, method);

    % Smoothening
    span1 = 0.12;
    span2 = 0.08;

    method = 'rlowess';
    pos_smooth = zeros(size(pos_interp));
    pos_smooth(:,1) = smooth(tnom,pos_interp(:,1),span1, method);
    pos_smooth(:,2) = smooth(tnom,pos_interp(:,2),span1, method);
    pos_smooth(:,3) = smooth(tnom,pos_interp(:,3),span1, method);

    orient_smooth = zeros(size(orient_interp));
    orient_smooth(:,1) = smooth(tnom,orient_interp(:,1),span2, method);
    orient_smooth(:,2) = smooth(tnom,orient_interp(:,2),span2, method);
    orient_smooth(:,3) = smooth(tnom,orient_interp(:,3),span2, method);
    orient_smooth(:,4) = smooth(tnom,orient_interp(:,4),span2, method);
%     orient_smooth(:,1) = sqrt(1-sum(orient_smooth(:,2:3).^2,2));

    obs.pose123.time_steps = tnom;
    obs.pose123.position = pos_smooth;
    obs.pose123.orientation = orient_smooth;
    
    % Get Linear Velocity
    obs = get_velocity(obs,fs);
    % Get Linear Acceleration
    obs = get_acceleration(obs,fs);
    % Get Fsum, Spatial and Body Frames
    obs = get_fsum(obs);
    
    % TODO:
    % Get angular velocity
end



function obs = outlier_removal(obs)

    obs.pose123.orientation(:,4) = unwrap(obs.pose123.orientation(:,4));
    
    % Extreme outlier removal from z-axis of orientation;
    % Assumption: Tray is not tilted for more than 45 deg from vertical
    % axis
    I1 = abs(obs.pose123.orientation(:,1)) > sqrt(2)/2;
    I2 = abs(obs.pose123.orientation(:,2)) > sqrt(2)/2;
    I3 = obs.pose123.orientation(:,3) < sqrt(2)/2 | obs.pose123.orientation(:,3) > 1.;
    I = I1 | I2 | I3;
    obs.pose123.time_steps = obs.pose123.time_steps(~I);
    obs.pose123.orientation = obs.pose123.orientation(~I,:);
    obs.pose123.position = obs.pose123.position(~I,:);
    
    % Box quantile based outlier removal
    [~,I1] = rmoutlier_quantile(obs.pose123.orientation(:,1:3), [2.2, 2.2, 2]);
    [~,I2] = rmoutlier_quantile(obs.pose123.position(:,:), [4., 2.8, 2.1]);
    I = I1 | I2;
    % Update the data
    obs.pose123.time_steps = obs.pose123.time_steps(~I);
    obs.pose123.orientation = obs.pose123.orientation(~I,:);
    obs.pose123.position = obs.pose123.position(~I,:);
    % Refine the angle ranges
    obs.pose123.orientation = quat2axang(axang2quat(obs.pose123.orientation));
    obs.pose123.orientation = obs.pose123.orientation.*sign(obs.pose123.orientation(:,3));
    obs.pose123.orientation(:,4) = unwrap(obs.pose123.orientation(:,4));
    

end

function obs = get_fsum(obs)
    % Fsum, Fstretch in Body and Spatial frames
    axangs = interp1(obs.pose123.time_steps, obs.pose123.orientation, obs.rft1.time_steps);

    rotm = axang2rotm(axangs);
    
    force_s = zeros(size(obs.rft1.force));
    force_s_1 = zeros(size(obs.rft1.force));
    torque_s = zeros(size(obs.rft1.torque));
    torque_s_1 = zeros(size(obs.rft1.torque));

    for i=1:length(axangs)

        force_s(i,:) = rotm(:,:,i)*obs.rft1.force(i,:)';
        force_s_1(i,:) = rotm(:,:,i)*obs.rft2.force(i,:)';

        torque_s(i,:) = rotm(:,:,i)*obs.rft1.torque(i,:)';
        torque_s_1(i,:) = rotm(:,:,i)*obs.rft2.torque(i,:)';

    end
    
    
    obs.rft1.forceS = force_s;
    obs.rft1.torqueS = torque_s;
    
    obs.rft2.forceS = force_s_1;
    obs.rft2.torqueS = torque_s_1;
    
    obs.fsum.force = obs.rft1.force + obs.rft2.force;
    obs.fsum.forceS = obs.rft1.forceS + obs.rft2.forceS;
    obs.fsum.time_steps = obs.rft1.time_steps;
    
    obs.fstretch.force = obs.rft1.force - obs.rft2.force;
    obs.fstretch.forceS = obs.rft1.forceS - obs.rft2.forceS;
    obs.fstretch.time_steps = obs.rft1.time_steps;
end


function obs = get_velocity(obs,fs)

    vel = diff(obs.pose123.position)*fs;
    vel = [0 0 0; vel];
      
    vel_smooth = medfilt1(vel,5);
    
    obs.pose123.linvel = vel_smooth;
    
    % Get Body Velocity
    rotm = axang2rotm(obs.pose123.orientation);
    
    velB = zeros(size(obs.pose123.linvel));
    for i=1:length(velB)
        velB(i,:) = rotm(:,:,i)'*obs.pose123.linvel(i,:)';
    end
    obs.pose123.linvelB = velB;
end

function obs = get_acceleration(obs,fs)

    acc = diff(obs.pose123.linvel)*fs;
    acc = [acc; 0 0 0];
      
    acc_smooth = medfilt1(acc,5);
    
    obs.pose123.linacc = acc_smooth;
    
    % Get Body Acceleration
    rotm = axang2rotm(obs.pose123.orientation);
    
    accB = zeros(size(obs.pose123.linacc));
    for i=1:length(accB)
        accB(i,:) = rotm(:,:,i)'*obs.pose123.linacc(i,:)';
    end
    obs.pose123.linaccB = accB;
end





