

clc;clear;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);

load('../../data/events/DetectedEvents_0.5sec.mat');

DetectedEvents_original = DetectedEvents;
DetectedEvents = event_correction(DetectedEvents, observations_processed);

[features] = haptic_features(observations_processed);


% st = struct2table(DetectedEvents);
% st = convertvars(st, {'obs_id'}, 'string');
% M = containers.Map(st.obs_id, st.events);




%% Plot Forces 

% Select an example
st = struct2table(DetectedEvents);
st = convertvars(st, {'obs_id'}, 'string');
M = containers.Map(st.obs_id, st.events);

ind = 29;
obs = observations_processed(ind);

feat = features(ind);
win_locs = M(obs.obs_id);


% Mix plot with accel
I = feat.angles.time_steps >= win_locs(1,1) & feat.angles.time_steps <= obs.tdec_sec;
tang = feat.angles.time_steps(I);
fstr = obs.fstretch.force(1:10:end,1:2); fstr = fstr(I,:);
fsum = obs.fsum.force(1:10:end,:); fsum = fsum(I,:);
f1 = obs.rft1.force(1:10:end,1:2); f1 = f1(I,:);
f2 = obs.rft2.force(1:10:end,1:2); f2 = f2(I,:);
tau1 = obs.rft1.ttorque(1:10:end,3); tau1 = tau1(I,:);
tau2 = obs.rft2.ttorque(1:10:end,3); tau2 = tau2(I,:);
accel = obs.imu.accelS(I,1:2)+0.5;

orient = rad2deg(obs.imu.orientation(I,4));
orient = orient - orient(1);

tmax = obs.tdec_sec + 0.25;
tmin = max(0,win_locs(1,1) - 0.25);

figure(1);
plot(tang, fstr(:,1), '--', 'LineWidth', 1.1, 'Color', [0.9 0 0]);  hold on;
plot(tang, f1(:,1), 'b', 'LineWidth', .7);
plot(tang, f2(:,1), 'g', 'LineWidth', .7);
plot(tang, fsum(:,1), '--', 'LineWidth', 1., 'Color', [0.5 0.5 0.5]);
grid on;
box on;

ymax = max(10,max(abs(fstr(:))+0.4));
ylim([-10, 8]);
xlim([tmin, tmax]);
% text(2.4, 5.6, 'F_{str}', 'FontSize', 14, 'Color', [0.8 0 0])
% text(1.5, -4, 'F_{1}', 'FontSize', 14, 'Color', 'blue')
% text(2., 2.8, 'F_{2}', 'FontSize', 14, 'Color', 'green')
%     for i = 1:size(win_locs,1)
%             a = xline(win_locs(i,1),'-.k', 't0');
%             a = xline(win_locs(i,2),'-.k', 'tf','LabelVerticalAlignment','bottom');
%     end
%     for i = 1:size(win_locs,1)
%             a = area(win_locs(i,:), [8 8], -8);
%     end

legend({'F_{str}', 'F_1', 'F_2', 'F_{sum}'}, 'NumColumns', 4, 'Location', 'North')
hold off; 
ylabel('[N]');
xlabel('time [sec]');



%% Event segmentation example MultiModal

% Select an example
st = struct2table(DetectedEvents);
st = convertvars(st, {'obs_id'}, 'string');
M = containers.Map(st.obs_id, st.events);

ind = 17;
obs = observations_processed(ind);

feat = features(ind);
win_locs = M(obs.obs_id);


% Mix plot with Force Displacement
I = feat.angles.time_steps >= win_locs(1,1) & feat.angles.time_steps <= obs.tdec_sec;
tang = feat.angles.time_steps(I);
fstr = obs.fstretch.force(1:10:end,1:2); fstr = fstr(I,:);
fsum = obs.fsum.forceS(1:10:end,:); fsum = fsum(I,:);
f1 = obs.rft1.forceS(1:10:end,1:2); f1 = f1(I,:);
f2 = obs.rft2.forceS(1:10:end,1:2); f2 = f2(I,:);
tau1 = obs.rft1.ttorqueS(1:10:end,3); tau1 = tau1(I,:);
tau2 = obs.rft2.ttorqueS(1:10:end,3); tau2 = tau2(I,:);
accel = obs.imu.accelS(I,1:2)+0.5;
pos = vecnorm(obs.pose123.position(I,1:2),2,2);
% pos_win_locs = get_pos_win_locs(pos, tang, win_locs);

orient = rad2deg(obs.imu.orientation(I,4));
orient = orient - orient(1);

tmax = obs.tdec_sec + 0.25;
tmin = max(0,win_locs(1,1) - 0.25);

srows = 3;
scols = 1;
color_map = summer(size(win_locs,1));

figure(2)
% First Column
subplot(srows, scols, 1);
a1 = plot(tang, feat.angles.angles(I,2),'b'); hold on; 
a2 = plot(tang, feat.angles.angles(I,3), 'm'); grid on; 
ylim([0 180]);
xlim([tmin, tmax]);
box off;
for i = 1:size(win_locs,1)
    a = area(win_locs(i,:), [180 180], 'FaceColor', color_map(i,:));
    a.FaceAlpha = 0.2;
    a.EdgeAlpha = 0.2;
    a.BaseLine.Visible = false;
end
ylabel('angle [deg]')
hold off;
legend('\alpha_1', '\alpha_2')

subplot(srows, scols, 2);
plot(tang, fstr(:,1), 'b'); grid on; hold on;
ymax = max(20,max(abs(fstr(:))+0.4));
ylim([-ymax, ymax]);
xlim([tmin, tmax]);
for i = 1:size(win_locs,1)
    a = area(win_locs(i,:), [ymax ymax], -ymax, 'FaceColor', color_map(i,:));
    a.FaceAlpha = 0.2;
    a.EdgeAlpha = 0.2;
    a.BaseLine.Visible = false;
end
legend('F_{str}')
ylabel('Force [N]')
box off;
hold off;

subplot(srows, scols, 3);
plot(tang, obs.pose123.linvel(I,1), 'b', tang, obs.pose123.linvel(I,2), 'm'); 
hold on; grid on;
ymax = max(0.5, max(max(abs(obs.pose123.linvel(I,:)))));
ylim([-ymax ymax]);
xlim([tmin, tmax]);
for i = 1:size(win_locs,1)
    a = area(win_locs(i,:), [ymax ymax], -ymax, 'FaceColor', color_map(i,:));
    a.FaceAlpha = 0.2;
    a.EdgeAlpha = 0.2;
    a.BaseLine.Visible = false;
end
ylabel('velocity [m/s]')
box off;
hold off;
xlabel('time [sec]')
legend({'v_x', 'v_y'}, 'Location', 'SouthEast')


% subplot(srows, scols, 2);
% plot(tang, fstr(:,1), 'b'); grid on; hold on;
%  hold off;
% ymax = max(20,max(abs(fstr(:))+0.4));
% ylim([-ymax, ymax]);
% xlim([tmin, tmax]);
% 
% for i = 1:size(win_locs,1)
%     a = xline(win_locs(i,1),'-.k');
%     a = xline(win_locs(i,2),'-.k');
% end
% box off;
% hold off;

%% Primitive Clusters

ks = unique(primtable.cluster);

ncols = numel(ks);
nrows = 2;

figure(300);
for j = 1:numel(ks)
    
    sl = primtable(primtable.cluster==ks(j),:);
    
    [hist_mat_f,hist_mat_v, fstr_label, vel_label, tlabel] ...
        = signal_histogram(sl);
    
    subplot(nrows,ncols,j);
    
    h = heatmap(hist_mat_f, 'Colormap', jet);
    title(sprintf("cluster %d ", j))
    h.XDisplayLabels = tlabel;
    h.YDisplayLabels = fstr_label;
    
    
    subplot(nrows, ncols,j+ncols);
    h = heatmap(hist_mat_v, 'Colormap', jet);
    h.XDisplayLabels = tlabel;
    h.YDisplayLabels = vel_label;
    
end
sgtitle('Primitive Clusters');





figure(401);

for j = 1:numel(ks)
    
    sl = primtable(primtable.cluster==ks(j),:);

    subplot(nrows,ncols,j);
    for i=1:size(sl,1)
       prim = sl.prim(i); 
       t = prim.time_steps - prim.time_steps(1);
       t = t/t(end);
       
       if sl.exec_prim(i)
            plot(t, prim.fstr(1:numel(t)),'r', 'LineWidth', 0.5);
            hold on;
            continue;
       end
       
       plot(t, prim.fstr(1:numel(t)),'b', 'LineWidth', 0.5);
       hold on;
       
    end
    
    ylim([-20, 40])
    hold off; grid on;
%     ylabel('F_{str}')
    subtitle('F_{str}');
    
    subplot(nrows,ncols,ncols+j);
    for i=1:size(sl,1)
       prim = sl.prim(i); 
       t = prim.time_steps - prim.time_steps(1);
       t = t/t(end);

        if sl.exec_prim(i)
            plot(t, prim.vel(1:numel(t),2),'r', 'LineWidth', 0.5);
            hold on;
            continue;
        end
       
       plot(t, (prim.vel(1:numel(t),2)),'b', 'LineWidth', 0.5);
       hold on;
       
    end
    ylim([-0.7, 0.7])
    grid on;
    subtitle('v_{y}');
    hold off;
    
end
sgtitle(sprintf('cluster: %d #points: %d', ks(j), size(sl,1)));



% sprintf('cluster: %d #points: %d', ks(j), size(sl,1))
