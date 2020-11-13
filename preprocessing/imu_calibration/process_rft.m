function [tnom,pos_smooth, orient_smooth] = process_rft(tnom,trft,force,torque)
%SMOOTH_POSE Summary of this function goes here
%   Detailed explanation goes here

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

end



function [force, torque] = lowpass_filter(force, torque, fs)


end
