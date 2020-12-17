

clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);

observations_processed = adjust_time(observations_processed);

%% Force Plots
fig_path = '../../data/plots/plots_preprocessed_v2_1/force/';

figure('visible','off');
for ind=progress(1:numel(observations_processed),'Title', 'Force Plots')
    obs = observations_processed(ind);
    plot_rfts(obs,2)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'force', '.jpg'])
end


%% Kinematics and Stretch Force plots
fig_path = '../../data/plots/plots_preprocessed_v2_1/kinematics_fstretch/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'Kinematics_Stretch')
    
    obs = observations_processed(ind);
%     try
    plot_pose(obs,3)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])
%     catch
%         fprintf('1 error is found! \n')
%         fails = [fails;ind];
%     end
    
end


%% Kinematics and Orientation plots
fig_path = '../../data/plots/plots_preprocessed_v2_1/kinematics_orientation/';
figure('visible','off');
for ind=progress(1:numel(observations_processed),'Title', 'Kinematics_Orient')
    
    obs = observations_processed(ind);
    plot_pose(obs,2)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])
end


%% Force Vector Plots
obs = observations_processed(10);

figure(1);

for ind = 1:20:numel(obs.rft1.time_steps)
    ip = 1+round(ind/10);
    plotv(obs.rft1.forceS(ind,1:2)','-->r'); hold on;
    plotv(obs.rft2.forceS(ind,1:2)','-->g')
    plotv(obs.fsum.forceS(ind,1:2)','->b')
    plotv(5*obs.pose123.linvel(ip,1:2)','-.xc')
    
    hold off; grid on;
    legend('F_1', 'F_2', 'F_{sum}', 'vel');
    title(sprintf('%2.2d sec  %2.2d sec', obs.rft1.time_steps(ind), obs.pose123.time_steps(ip)))
    xlim([-10,10]);
    ylim([-10,10]);
    
    pause(0.01);
end


%% Quiver Vector plot Animation

rv1 = [ 0.2275, 0, -0.015];
rv2 = [-0.2275, 0, -0.015];
obs = observations_processed(10);

inds = 1:2:numel(obs.pose123.time_steps);

% Compute Handle coordinates
R = axang2rotm(obs.pose123.orientation(inds,:));
P = obs.pose123.position(inds,:);
H1 = zeros(size(P));
H2 = H1;
for j = 1:numel(inds)
    H1(j,:) = R(:,:,j)*rv1'+P(j,:)';
    H2(j,:) = R(:,:,j)*rv2'+P(j,:)';
end

for k = 1:numel(inds)
    k = inds(k);
    figure(2);

    tr = plot(obs.pose123.position(:,1), obs.pose123.position(:,2),'black','LineWidth',1.);
    grid on; hold on;

    % Velocity Vector
    ve = quiver(obs.pose123.position(k,1),obs.pose123.position(k,2),...
        obs.pose123.linvel(k,1),obs.pose123.linvel(k,2), 'r', 'MaxHeadSize',0.05,...
        'DisplayName','velocity');
    ac = quiver(obs.pose123.position(k,1),obs.pose123.position(k,2),...
        obs.pose123.linacc(k,1),obs.pose123.linacc(k,2), 'c', 'MaxHeadSize',0.05,...
        'DisplayName','accel');

    % Points of Interest
    scatter(obs.pose123.position(k,1),obs.pose123.position(k,2),'blue','filled')
    scatter(H1(k,1),H1(k,2),'m','filled')
    scatter(H2(k,1),H2(k,2),'p','filled')
    % for ii = 1:length(H1)
    plot([H1(k,1), H2(k,1)],[H1(k,2), H2(k,2)], 'k','LineWidth',.5)
    % end

    % Force1 Vector
    f1 = quiver(H1(k,1),H1(k,2), obs.rft1.forceS(k*10,1),obs.rft1.forceS(k*10,2),...
        '-.m','LineWidth',1.5, 'MaxHeadSize',0.05, 'DisplayName', 'F_1');
    % Force2 Vector
    f2 = quiver(H2(k,1),H2(k,2), obs.rft2.forceS(k*10,1),obs.rft2.forceS(k*10,2),...
        '-.p','LineWidth',1.5,'ShowArrowHead','on', 'MaxHeadSize',0.05, 'DisplayName', 'F_2');
    % Fsum Vector
    fsum = quiver(obs.pose123.position(k,1),obs.pose123.position(k,2),...
        obs.fsum.forceS(k*10,1),obs.fsum.forceS(k*10,2),'b','LineWidth',1.5,...
        'MaxHeadSize',0.05, 'DisplayName', 'F_{sum}');

    xlabel('x-axis, m');
    ylabel('y-axis, m');
    % subtitle('2D Trajectory')
    xts = xticks;
    % xl1 = xlim;
    xticks(xts(1):0.5:xts(end));
    rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',1)
    text(-0.05,-0.6, 'A', 'FontSize',14)
    rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',1)
    text( 2.65,-0.6, 'B', 'FontSize',14)
    rectangle('Position',[1.25 -0.2 .25 .3], 'EdgeColor', 'black', 'LineWidth',3)
    text( 1.35,-0.05, 'Obs', 'FontSize',12)

    hold off;
    xlim([-0.5,3.1]);
    ylim([-1.5,1.5]);

    legend([tr,ve,ac,f1,f2,fsum], {'trajectory', 'velocity', 'accel','F_1','F_2','F_{sum}'});
    % legend;
    sgtitle(sprintf('%s    %s    %s    Kinematics', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');
    pause(0.01)
end


%% Quiver Vector plot

rv1 = [ 0.2275, 0, -0.015];
rv2 = [-0.2275, 0, -0.015];
obs = observations_processed(10);

inds = 1:40:numel(obs.pose123.time_steps);

% Compute Handle coordinates
R = axang2rotm(obs.pose123.orientation(inds,:));
P = obs.pose123.position(inds,:);
H1 = zeros(size(P));
H2 = H1;
for j = 1:numel(inds)
    H1(j,:) = R(:,:,j)*rv1'+P(j,:)';
    H2(j,:) = R(:,:,j)*rv2'+P(j,:)';
end

figure(2);

tr = plot(obs.pose123.position(:,1), obs.pose123.position(:,2),'black','LineWidth',1.);
grid on; hold on;

% Velocity Vector
ve = quiver(obs.pose123.position(inds,1),obs.pose123.position(inds,2),...
    obs.pose123.linvel(inds,1),obs.pose123.linvel(inds,2), 'r', 'MaxHeadSize',0.05,...
    'DisplayName','velocity');
ac = quiver(obs.pose123.position(inds,1),obs.pose123.position(inds,2),...
    obs.pose123.linacc(inds,1),obs.pose123.linacc(inds,2), 'c', 'MaxHeadSize',0.05,...
    'DisplayName','accel');

% Points of Interest
scatter(obs.pose123.position(inds,1),obs.pose123.position(inds,2),'blue','filled')
scatter(H1(:,1),H1(:,2),'m','filled')
scatter(H2(:,1),H2(:,2),'p','filled')
for ii = 1:length(H1)
plot([H1(ii,1), H2(ii,1)],[H1(ii,2), H2(ii,2)], 'k','LineWidth',.5)
end

% Force1 Vector
f1 = quiver(H1(:,1),H1(:,2), obs.rft1.forceS(inds*10,1),obs.rft1.forceS(inds*10,2),...
    '-.m','LineWidth',1.5, 'MaxHeadSize',0.05, 'DisplayName', 'F_1');
% Force2 Vector
f2 = quiver(H2(:,1),H2(:,2), obs.rft2.forceS(inds*10,1),obs.rft2.forceS(inds*10,2),...
    '-.p','LineWidth',1.5,'ShowArrowHead','on', 'MaxHeadSize',0.05, 'DisplayName', 'F_2');
% Fsum Vector
fsum = quiver(obs.pose123.position(inds,1),obs.pose123.position(inds,2),...
    obs.fsum.forceS(inds*10,1),obs.fsum.forceS(inds*10,2),'b','LineWidth',1.5,...
    'MaxHeadSize',0.05, 'DisplayName', 'F_{sum}');

xlabel('x-axis, m');
ylabel('y-axis, m');
% subtitle('2D Trajectory')
xts = xticks;
xl1 = xlim;
xticks(xts(1):0.5:xts(end));
rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',1)
text(-0.05,-0.6, 'A', 'FontSize',14)
rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',1)
text( 2.65,-0.6, 'B', 'FontSize',14)
rectangle('Position',[1.25 -0.2 .25 .3], 'EdgeColor', 'black', 'LineWidth',3)
text( 1.35,-0.05, 'Obs', 'FontSize',12)

hold off;
xlim([-0.5,3.1]);
ylim([-1.5,1.5]);

legend([tr,ve,ac,f1,f2,fsum], {'trajectory', 'velocity', 'accel','F_1','F_2','F_{sum}'});
% legend;
sgtitle(sprintf('%s    %s    %s    Kinematics', obs.obs_id, obs.traj_type, obs.motion_type ), ...
        'Interpreter','none');

%% Quiver Vector Plot Function


fig_path = '../../data/plots/plots_preprocessed_v2_1/trajectory_vector_body_fr/';
figure('visible','off');
for ind=progress(1:numel(observations_processed), 'Title', 'TrajVectorPlot')
    
    obs = observations_processed(ind);

    trajvector_plot(obs,2)
    saveas(gcf, [fig_path, obs.obs_id, '_', obs.traj_type, '_', obs.motion_type,'_', 'kinematics', '.jpg'])

end
%%

obs = observations_processed(2);
figure(1)
plot_phase(obs,1)


