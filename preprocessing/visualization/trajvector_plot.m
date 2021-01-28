function [] = trajvector_plot(obs, opt)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
step_ = 40; % 1e-1 seconds

rv1 = [ 0.235, 0, -0.027];
rv2 = [-0.235, 0, -0.027];


inds = 1:step_:numel(obs.pose123.time_steps);
rftinds = 1:step_*10:numel(obs.pose123.time_steps)*10;

% Compute Handle coordinates
R = axang2rotm(obs.pose123.orientation(inds,:));
P = obs.pose123.position(inds,:);
H1 = zeros(size(P));
H2 = H1;
for j = 1:numel(inds)
    H1(j,:) = R(:,:,j)*rv1'+P(j,:)';
    H2(j,:) = R(:,:,j)*rv2'+P(j,:)';
end

tr = plot(obs.pose123.position(:,1), obs.pose123.position(:,2),'black','LineWidth',1.);
grid on; hold on;

% Velocity Vector
ve = quiver(obs.pose123.position(inds,1),obs.pose123.position(inds,2),...
    obs.pose123.linvel(inds,1),obs.pose123.linvel(inds,2), 'r', 'MaxHeadSize',0.05,...
    'DisplayName','velocity');


% Points of Interest
scatter(obs.pose123.position(inds,1),obs.pose123.position(inds,2),4.5,'blue','filled','MarkerFaceAlpha',0.5)
scatter(H1(:,1),H1(:,2),3,'m','filled','MarkerFaceAlpha',0.5)
scatter(H2(:,1),H2(:,2),3,'g','filled','MarkerFaceAlpha',0.5)

for ii = 1:length(H1)
plot([H1(ii,1), H2(ii,1)],[H1(ii,2), H2(ii,2)], 'k','LineWidth', .01)
% alpha(0.1)
end
        


switch opt
    case 1
        % RFT in Spatial Frame


        % Force1 Vector
        f1 = quiver(H1(:,1),H1(:,2), obs.rft1.forceS(rftinds,1),obs.rft1.forceS(rftinds,2),...
            'm','LineWidth',1.5, 'MaxHeadSize',0.05,'ShowArrowHead','on');
        % Force2 Vector
        f2 = quiver(H2(:,1),H2(:,2), obs.rft2.forceS(rftinds,1),obs.rft2.forceS(rftinds,2),...
            'g','LineWidth',1.5,'ShowArrowHead','on', 'MaxHeadSize',0.05);
        % Fsum Vector
        fsum = quiver(obs.pose123.position(inds,1),obs.pose123.position(inds,2),...
            obs.fsum.forceS(rftinds,1),obs.fsum.forceS(rftinds,2),'b','LineWidth',1.5,...
            'MaxHeadSize',0.05);

        ac = quiver(obs.pose123.position(inds,1),obs.pose123.position(inds,2),...
            obs.pose123.linacc(inds,1),obs.pose123.linacc(inds,2), 'c', 'MaxHeadSize',0.05,...
            'DisplayName','accel');

    case 2
        % RFT and imu.accel in Body Frame
        % Force1 Vector
        f1 = quiver(H1(:,1),H1(:,2), obs.rft1.force(rftinds,1),obs.rft1.force(rftinds,2),...
            'm','LineWidth',1.5, 'MaxHeadSize',0.05,'ShowArrowHead','on');
        % Force2 Vector
        f2 = quiver(H2(:,1),H2(:,2), obs.rft2.force(rftinds,1),obs.rft2.force(rftinds,2),...
            'g','LineWidth',1.5,'ShowArrowHead','on', 'MaxHeadSize',0.05);
        % Fsum Vector
        fsum = quiver(obs.pose123.position(inds,1),obs.pose123.position(inds,2),...
            obs.fsum.force(rftinds,1),obs.fsum.force(rftinds,2),'b','LineWidth',1.5,...
            'MaxHeadSize',0.05);
        ac = quiver(obs.pose123.position(inds,1),obs.pose123.position(inds,2),...
            obs.imu.accel(inds,1),obs.imu.accel(inds,2), 'c', 'MaxHeadSize',0.05,...
            'DisplayName','accel');
        
end

% Plot Objects
rectangle('Position',[-0.2 -0.5 .4 1], 'EdgeColor', 'green', 'LineWidth',1)
text(-0.05,-0.6, 'A', 'FontSize',14)
rectangle('Position',[2.5 -0.5 .4 1], 'EdgeColor', 'red', 'LineWidth',1)
text( 2.65,-0.6, 'B', 'FontSize',14)
rectangle('Position',[1.25 -0.2 .25 .3], 'EdgeColor', 'black', 'LineWidth',3)
text( 1.35,-0.05, 'Obs', 'FontSize',12)


% Title and axis labels
xlabel('x-axis, m');
ylabel('y-axis, m');

% subtitle('2D Trajectory')
xts = xticks;
% xl1 = xlim;
xticks(xts(1):0.5:xts(end));


hold off;
xlim([-0.5,3.1]);
% ylim([-1.5,1.5]);
axis equal;

legend([tr,ve,ac,f1,f2,fsum], {'trajectory', 'velocity', 'accel','F_1','F_2','F_{sum}'});
% legend;
sgtitle(sprintf('%s    %s    %s    Kinematics tlag:%.2d', obs.obs_id, obs.traj_type, obs.motion_type, ...
    mean(obs.pose123.time_steps(inds)-obs.rft1.time_steps(rftinds))), 'Interpreter','none');


end

