
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
subplot(221);
plot(obs.pose123.time_steps(I), obs.pose123.position(I,:));
grid on;

subplot(222);
plot(obs.pose123.time_steps(I), obs.pose123.orientation(I,:));
grid on;

subplot(223);
plot(obs.rft1.time_steps(If), obs.rft1.force(If,:));
grid on;

subplot(224);
plot(obs.rft2.time_steps(If), obs.rft2.force(If,:));
grid on;




%% Computed Torque

obs = observations_processed(2);

rv1 = [ 0.2275, 0, -0.015];
rv2 = [-0.2275, 0, -0.015];

tcomp1 = cross(obs.rft1.torque, repmat(rv1,[numel(obs.rft1.time_steps), 1]));
tcomp2 = cross(obs.rft2.torque, repmat(rv2,[numel(obs.rft2.time_steps), 1]));
obs.rft1.tcomp = tcomp1;
obs.rft2.tcomp = tcomp2;
obs.rft1.ttorque = obs.rft1.torque+tcomp1;
obs.rft2.ttorque = obs.rft2.torque+tcomp2;
obs.fsum.ttsum = obs.rft2.torque+tcomp2 + obs.rft1.torque+tcomp1;

figure(3); 
subplot(3,3,1);
plot(obs.rft1.time_steps, tcomp1);
subtitle('\tau^{c}_1'); grid on;

subplot(3,3,2);
plot(obs.rft1.time_steps, obs.rft1.torque);
subtitle('\tau^{m}_1'); grid on;

subplot(3,3,3);
plot(obs.rft1.time_steps, obs.rft1.torque+tcomp1);
subtitle('\tau^{tot}_1'); grid on;

subplot(3,3,4);
plot(obs.rft2.time_steps, tcomp2);
subtitle('\tau^{m}_2'); grid on;

subplot(3,3,5);
plot(obs.rft2.time_steps, obs.rft2.torque);
subtitle('\tau^{c}_2'); grid on;

subplot(3,3,6);
plot(obs.rft2.time_steps, obs.rft2.torque+tcomp2);
subtitle('\tau^{tot}_1'); grid on;

subplot(3,3,7);
plot(obs.rft2.time_steps, obs.rft2.torque+tcomp2 + obs.rft1.torque+tcomp1);
subtitle('\tau_{sum}'); grid on;

subplot(3,3,8);
plot(obs.rft2.time_steps, -obs.rft2.torque-tcomp2 + obs.rft1.torque+tcomp1);
subtitle('\tau_{stretch}'); grid on;

hL = legend('x','y','z','NumColumns',3);
set(hL, 'Position',[0.51 0.03 0.01 0.01],'Units','normalized')



%% Haptics Based features FORCES

dotsv = @(f1,f2) f1(:,1).*f2(:,1) + f1(:,2).*f2(:,2);

% obs = observations_processed(5);
ax = [1,2];
eps = 0.01;

Me1fxy = dotsv(obs.rft1.force(:,ax),obs.fsum.force(:,ax))...
 ./(vecnorm(obs.rft1.force(:,ax),2,2).*vecnorm(obs.fsum.force(:,ax),2,2));

Me2fxy = dotsv(obs.rft2.force(:,ax),obs.fsum.force(:,ax))...
 ./(vecnorm(obs.rft2.force(:,ax),2,2).*vecnorm(obs.fsum.force(:,ax),2,2));


Mtfxy = vecnorm(obs.fsum.force(:,ax),2,2)...
    ./(vecnorm(obs.rft1.force(:,ax),2,2)+vecnorm(obs.rft2.force(:,ax),2,2));

Mnfxy = vecnorm(obs.fsum.force(:,ax),2,2).^2 ...
    ./(abs(dotsv(obs.rft1.force(:,ax),obs.fsum.force(:,ax)))...
     + abs(dotsv(obs.rft2.force(:,ax),obs.fsum.force(:,ax))));

Msfxy = 1 - abs( (abs(dotsv(obs.rft1.force(:,ax),obs.fsum.force(:,ax)))...
     - abs(dotsv(obs.rft2.force(:,ax),obs.fsum.force(:,ax)))) ...
     ./vecnorm(obs.fsum.force(:,ax),2,2).^2);
 
 
figure(1);
subplot(411)
plot(obs.rft1.time_steps, Me1fxy); hold on;
plot(obs.rft1.time_steps, Me2fxy); hold off;
subtitle('Individual Efficiency')

subplot(412)
plot(obs.rft1.time_steps, Mtfxy);
subtitle('Team Efficiency')

% ylim([0,20])
subplot(413)
plot(obs.rft1.time_steps, Mnfxy);
subtitle('Negotiation Efficiency')


subplot(414)
plot(obs.rft1.time_steps, Msfxy);
subtitle('Similarity of Forces')
sgtitle('Haptic Features')


figure(2);
plot_rfts(obs,2)

%% Haptics Based features TORQUES

dotsv = @(f1,f2) f1(:,1).*f2(:,1) + f1(:,2).*f2(:,2);

% obs = observations_processed(5);
ax = [2,3];
eps = 0.01;

Me1txy = dotsv(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax))...
 ./(vecnorm(obs.rft1.torque(:,ax),2,2).*vecnorm(obs.fsum.ttsum(:,ax),2,2));

Me2txy = dotsv(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax))...
 ./(vecnorm(obs.rft2.ttorque(:,ax),2,2).*vecnorm(obs.fsum.ttsum(:,ax),2,2));


Mttxy = vecnorm(obs.fsum.ttsum(:,ax),2,2)...
    ./(vecnorm(obs.rft1.ttorque(:,ax),2,2)+vecnorm(obs.rft2.ttorque(:,ax),2,2));

Mntxy = vecnorm(obs.fsum.ttsum(:,ax),2,2).^2 ...
    ./(abs(dotsv(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax)))...
     + abs(dotsv(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax))));

Mstxy = 1 - abs( (abs(dotsv(obs.rft1.ttorque(:,ax),obs.fsum.ttsum(:,ax)))...
     - abs(dotsv(obs.rft2.ttorque(:,ax),obs.fsum.ttsum(:,ax)))) ...
     ./vecnorm(obs.fsum.ttsum(:,ax),2,2).^2);
 
 
figure(1);
subplot(411)
plot(obs.rft1.time_steps, Me1txy); hold on;
plot(obs.rft1.time_steps, Me2txy); hold off;
subtitle('Individual Efficiency')

subplot(412)
plot(obs.rft1.time_steps, Mttxy);
subtitle('Team Efficiency')

% ylim([0,20])
subplot(413)
plot(obs.rft1.time_steps, Mntxy);
subtitle('Negotiation Efficiency')


subplot(414)
plot(obs.rft1.time_steps, Mstxy);
subtitle('Similarity of Forces')
sgtitle('Haptic Features')


% figure(2);
% plot_rfts(obs,2)
