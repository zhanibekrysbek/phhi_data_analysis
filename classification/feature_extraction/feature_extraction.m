
clc;clear;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);


%% Compute the features

% obs = observations_processed(2);
% 
% figure(1);
% plot_rfts(obs, 2);


features(numel(observations_processed)) = struct('haptics',[], 'motion_type',[],...
            'obs_id',[], 'traj_type', []);
        
% profile on;
for i=progress(1:numel(features), 'Title','FeatureExtraction')
    
    obs = observations_processed(i);
    features(i).obs_id = obs.obs_id;
    features(i).traj_type = obs.traj_type;
    features(i).motion_type = obs.motion_type;
    
    features(i) = haptic_features(features(i), obs);
     
end
% profile viewer;

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

%% Sliding Window with Integration

feat = features(2);

numfeat = numel(fieldnames(features(1).haptics));
Z = zeros(numel(features), numfeat);
Y = zeros(numel(features),1);

t0 = 0.0;
tf = 2.0;
for i=progress(1:numel(features), 'Title','FeatureExtraction')
    obs = observations_processed(i);
    t0 = 0.0;
    tf = 0.6*obs.pose123.time_steps(end);
    [Z(i,1:end-1),Y(i)] = integrate_features(features(i),t0,tf);
    ind_p = obs.pose123.time_steps<=tf;
    yf = max(obs.pose123.position(ind_p,2));
    Z(i,end) = yf;
end

%% LDA and Plot

[Z1, W, lambda] = LDA(Z,Y);

figure(1);
plot3(Z1(Y==0,1),Z1(Y==0,2),Z1(Y==0,3), 'o'); hold on;
plot3(Z1(Y==1,1),Z1(Y==1,2),Z1(Y==1,3), 'x'); hold off; grid on;


%% Angle btwn F1 F2.

obs = observations_processed(3);

v = obs.pose123.linvel(:,[1,2]);
f1 = obs.rft1.forceS(1:10:end,1:2);
f2 = obs.rft2.forceS(1:10:end,1:2);
fsum = obs.fsum.forceS(1:10:end,1:2);
fstr = obs.fstretch.forceS(1:10:end,1:2);

theta1 = acosd(vecdot(f1,f2)./(vecnorm(f1,2,2).*vecnorm(f2,2,2)));
theta2 = acosd(vecdot(f1,v)./(vecnorm(f1,2,2).*vecnorm(v,2,2)));
theta3 = acosd(vecdot(f2,v)./(vecnorm(f2,2,2).*vecnorm(v,2,2)));
theta4 = acosd(vecdot(fsum,v)./(vecnorm(fsum,2,2).*vecnorm(v,2,2)));
theta5 = acosd(vecdot(fstr,v)./(vecnorm(fstr,2,2).*vecnorm(v,2,2)));
theta6 = acosd(vecdot(f1,fsum)./(vecnorm(f1,2,2).*vecnorm(fsum,2,2)));
theta7 = acosd(vecdot(f2,fsum)./(vecnorm(f2,2,2).*vecnorm(fsum,2,2)));
theta8 = acosd(vecdot(fstr,fsum)./(vecnorm(fstr,2,2).*vecnorm(fsum,2,2)));
theta9 = acosd(vecdot(f1,fstr)./(vecnorm(f1,2,2).*vecnorm(fstr,2,2)));
theta10 = acosd(vecdot(f2,fstr)./(vecnorm(f2,2,2).*vecnorm(fstr,2,2)));


Nsubs = 10;
figure(1);
subplot(Nsubs,1,1)
plot(obs.pose123.time_steps, theta1);
subtitle('f1 vs f2');
xline(obs.tdec_sec)

subplot(Nsubs,1,2)
plot(obs.pose123.time_steps, theta2);
subtitle('f1 vs v')

subplot(Nsubs,1,3)
plot(obs.pose123.time_steps, theta3);
subtitle('f2 vs v')
xline(obs.tdec_sec)

subplot(Nsubs,1,4)
plot(obs.pose123.time_steps, theta4);
subtitle('fsum vs v')

subplot(Nsubs,1,5)
plot(obs.pose123.time_steps, theta5);
subtitle('fstr vs v')
xline(obs.tdec_sec)

subplot(Nsubs,1,6)
plot(obs.pose123.time_steps, theta6);
subtitle('f1 vs fsum')
xline(obs.tdec_sec)

subplot(Nsubs,1,7)
plot(obs.pose123.time_steps, theta7);
subtitle('f2 vs fsum')

subplot(Nsubs,1,8)
plot(obs.pose123.time_steps, theta8);
subtitle('fstr vs fsum')
xline(obs.tdec_sec)

subplot(Nsubs,1,9)
plot(obs.pose123.time_steps, theta9);
subtitle('f1 vs fstr')
xline(obs.tdec_sec)

subplot(Nsubs,1,10)
plot(obs.pose123.time_steps, theta10);
subtitle('f2 vs fstr')
xline(obs.tdec_sec)

figure(2);
plot_rfts(obs,1);


%% integration of Fstretch


obs = observations_processed(3);

f1 = obs.rft1.forceS(1:10:end,1);
f2 = obs.rft2.forceS(1:10:end,1);
fsum = obs.fsum.forceS(1:10:end,1:2);
fstr = obs.fstretch.force(1:10:end,1);

x = obs.pose123.position(:,1);
y = obs.pose123.position(:,2);

integx = cumtrapz(x,fstr);
integt = cumtrapz(obs.pose123.time_steps,fstr);


figure(3);

subplot(221);
plot(obs.pose123.time_steps, fstr); grid on;
xline(obs.tdec_sec)

subplot(223);
plot(x, fstr); grid on;

subplot(224);
plot(obs.pose123.time_steps, integx); grid on;

subplot(222);
plot(obs.pose123.time_steps, integt); grid on;
xline(obs.tdec_sec)


figure(2);
plot_rfts(obs,1);






