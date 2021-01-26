
clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);
% observations_processed = adjust_time(observations_processed);


%% Compute the features

% obs = observations_processed(2);
% 
% figure(1);
% plot_rfts(obs, 2);

ax = [1,2];

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


%% Sliding Window


wind_size = 0.05;
stride = 0.02;
divisions = 5;

t0 = 0;
Nwinds = ceil((1 - wind_size)/stride);

X = zeros(Nwinds*numel(observations_processed), divisions*28);
Y = zeros(Nwinds*numel(observations_processed), 3);

for ind=progress(1:numel(observations_processed))
    obs = observations_processed(ind);
    [X((ind-1)*Nwinds+1:ind*Nwinds,:), Y((ind-1)*Nwinds+1:ind*Nwinds,:)] = ...
        sliding_window(obs, wind_size, stride, divisions);
end



%% Debugging Sliding Window

obs = observations_processed(2);

wind_size = 0.05;
stride = 0.02;
divisions = 5;

X = sliding_window(obs, wind_size, stride, divisions);












