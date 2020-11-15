


% ld = load('/home/zhanibek/codes/phhi_data_analysis/data/obs_example_4.mat');

obs = observations(10);

tpos = obs.pose123.time_steps;
position = obs.pose123.position;
orientation = obs.pose123.orientation;

tpos = [0 tpos];
position = [position(1,:); position];
orientation = [orientation(1,:);orientation];
%% Interpolation
fs = 100;
tnom = 0:1/fs:tpos(end);

method = 'pchip';
pos_interp = interp1(tpos,position,tnom, method);

orient_interp = interp1(tpos,orientation,tnom, method);

figure(33);

subplot(211)
plot(tnom, pos_interp,'-'); grid on;
hold on;
plot(tpos, position,'x'); grid on;
hold off;
ylabel('m')
subtitle('position')
title('Interpolation')

legend('x-interp','y-interp','z-interp','x-orig','y-orig','z-orig')


subplot(212)
plot(tnom, orient_interp,'-.'); grid on;
hold on;
plot(tpos, orientation,'x'); grid on;
hold off;
xlabel('time, sec')
ylabel('axis-angle')
subtitle('orientation')



%% Smoothening
span1 = 0.15;
span2 = 0.06;

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


figure(10);
title('Smoothening')
subplot(2,1,1)
plot(tnom,pos_smooth,'LineWidth',2);hold on;
plot(tpos,position,'x');hold off;
grid on;
xlabel('time, sec')
ylabel('m')
legend('x','y','z', 'x-meas', 'y-meas','z-meas')
subtitle('position')
title('Smoothening')


subplot(2,1,2)
plot(tnom,orient_smooth,'LineWidth',2);hold on;
plot(tpos,orientation,'x');hold off;
grid on;
xlabel('time, sec')
ylabel('axis-angle')
legend('x','y','z', 'angle', 'x-meas', 'y-meas','z-meas','angle-meas')
subtitle('orientation')
