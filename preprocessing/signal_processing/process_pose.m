function [obs] = process_pose(obs, tf)
%process_pose Summary of this function goes here
%   Detailed explanation goes here

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
    span1 = 0.1;
    span2 = 0.05;

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

    obs.pose123.time_steps = tnom;
    obs.pose123.position = pos_smooth;
    obs.pose123.orientation = orient_smooth;
    
    
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





