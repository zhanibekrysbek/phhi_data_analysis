
clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);
observations_processed = adjust_time(observations_processed);


%% Choose time interval

obs = observations_processed(2);

figure(1); 
plot_rfts(obs, 2);


%% Indices

I = obs.pose123.position(:,2) <= 0.2 & obs.pose123.position(:,1)<=1.6;

tf = obs.pose123.time_steps(I);
tf = tf(end);

If = obs.rft1.time_steps<=tf;



figure(2);
subplot(221)
plot(obs.pose123.time_steps(I), obs.pose123.position(I,:))
grid on;

subplot(222)
plot(obs.pose123.time_steps(I), obs.pose123.orientation(I,:))
grid on;

subplot(223)
plot(obs.rft1.time_steps(If), obs.rft1.force(If,:))
grid on;

subplot(224)
plot(obs.rft2.time_steps(If), obs.rft2.force(If,:))
grid on;

%% Force Vector Plots
obs = observations_processed(80);

figure(1);

for ind = 1:20:numel(obs.rft1.time_steps)
    ip = 1+round(ind/10);
    plotv(obs.rft1.forceS(ind,1:2)','-->r'); hold on;
    plotv(obs.rft2.forceS(ind,1:2)','-->g')
    plotv(obs.fsum.forceS(ind,1:2)','->b')
    plotv(5*obs.pose123.linvel(ip,1:2)','-.xc')
    
    hold off;grid on;
    legend('F_1', 'F_2', 'F_{sum}', 'vel')
    title(sprintf('%2.2d sec  %2.2d sec', obs.rft1.time_steps(ind), obs.pose123.time_steps(ip)))
    xlim([-7,7])
    ylim([-7,7])
    
    pause(0.01)
end

%% Computed Torque

obs = observations_processed(10);


rv1 = [0.2275, 0, -0.015];
rv2 = [-0.2275, 0, -0.015];

tcomp1 = cross(obs.rft1.torque, repmat(rv1,[numel(obs.rft1.time_steps), 1]));
tcomp2 = cross(obs.rft2.torque, repmat(rv2,[numel(obs.rft2.time_steps), 1]));

figure(3); 
subplot(3,3,1);
plot(obs.rft1.time_steps, tcomp1); grid on;
subtitle('\tau^{c}_1')

subplot(3,3,2);
plot(obs.rft1.time_steps, obs.rft1.torque); grid on;
subtitle('\tau^{m}_1');

subplot(3,3,3);
plot(obs.rft1.time_steps, obs.rft1.torque+tcomp1); grid on;
subtitle('\tau^{tot}_1');



subplot(3,3,4);
plot(obs.rft2.time_steps, tcomp2); grid on;
subtitle('\tau^{m}_2');

subplot(3,3,5);
plot(obs.rft2.time_steps, obs.rft2.torque); grid on;
subtitle('\tau^{c}_2');

subplot(3,3,6);
plot(obs.rft2.time_steps, obs.rft2.torque+tcomp2); grid on;
subtitle('\tau^{tot}_1');


subplot(3,3,7);
plot(obs.rft2.time_steps, obs.rft2.torque+tcomp2+obs.rft1.torque+tcomp1); grid on;
subtitle('\tau_{sum}');

subplot(3,3,8);
plot(obs.rft2.time_steps, -obs.rft2.torque-tcomp2+obs.rft1.torque+tcomp1); grid on;
subtitle('\tau_{stretch}');

legend('x','y','z')
