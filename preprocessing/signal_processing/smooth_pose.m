function [tnom,pos_smooth, orient_smooth] = smooth_pose(tpos,position,orientation)
%SMOOTH_POSE Summary of this function goes here
%   Detailed explanation goes here

tpos = [0 tpos];
position = [0 0 0; position];
orientation = [orientation(1,:);orientation];

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


% figure(10);
% title('Smoothening')
% subplot(2,1,1)
% plot(tnom,pos_smooth);hold on;
% plot(tnom,pos_interp,'.');hold off;
% grid on;
% xlabel('time, sec')
% ylabel('m')
% legend('x','y','z', 'x-meas', 'y-meas','z-meas')
% subtitle('position')
% title('Smoothening')
% 
% 
% subplot(2,1,2)
% plot(tnom,orient_smooth);hold on;
% plot(tnom,orient_interp,'.');hold off;
% grid on;
% xlabel('time, sec')
% ylabel('axis-angle')
% legend('x','y','z', 'angle', 'x-meas', 'y-meas','z-meas','angle-meas')
% subtitle('orientation')

end

