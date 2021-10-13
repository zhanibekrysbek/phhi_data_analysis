

clc;clear;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);

load('../../data/events/DetectedEvents_v2.mat');

DetectedEvents_original = DetectedEvents;
DetectedEvents = event_correction(DetectedEvents, observations_processed);

[features] = haptic_features(observations_processed);


load ../../data/final_clusters.mat;

% st = struct2table(DetectedEvents);
% st = convertvars(st, {'obs_id'}, 'string');
% M = containers.Map(st.obs_id, st.events);

%% Cluster Correction:

% primid: 97,  from 3, to 5
% .      199,  from 3, to 6
%       :154,  from 2, to 6

% cluster_names = {'C1', 'C2', 'C3', 'C4', 'C5', 'C6'};

cluster_names = {'DR', 'NT', 'NC', 'DL', 'EL', 'ER'};

primtable.cluster(primtable.prim_id==97) = 5;
primtable.cluster(primtable.prim_id==199) = 6;
primtable.cluster(primtable.prim_id==154) = 6;


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

tmax = obs.tdec_sec; % 0.25;
tmin = max(1, win_locs(1,1) - 0.25);

figure(1);
plot(tang, f1(:,1), 'b', 'LineWidth', 1.1); hold on;
plot(tang, f2(:,1), 'g', 'LineWidth', 1.1);
plot(tang, fstr(:,1), '--', 'LineWidth', 1.5, 'Color', [0.9 0 0]);
plot(tang, fsum(:,1), '--', 'LineWidth', 1.5, 'Color', [0.3 0.3 0.3]);
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

legend({'F_1', 'F_2', 'F_{str}', 'F_{sum}'}, 'NumColumns', 4,...
    'Location', 'North', 'FontSize', 13)
hold off; 
ylabel('[N]', 'FontSize', 13);
xlabel('time [sec]', 'FontSize', 13);
xticks((1:4));
yticks([-10, -5, 0, 5, 8])



%% Event segmentation example MultiModal

% Select an example
st = struct2table(DetectedEvents);
st = convertvars(st, {'obs_id'}, 'string');
M = containers.Map(st.obs_id, st.events);


ind = 17;
obs = observations_processed(ind);

feat = features(ind);
win_locs = M(obs.obs_id);

seq = primtable.cluster(primtable.obs_ind==ind);
lfcons = primtable.lfcons(primtable.obs_ind==ind);


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

tmax = obs.tdec_sec + 0.06;
tmin = win_locs(1,1) - 0.06;
% tmin = min(0.75, win_locs(1,1));

srows = 3;
scols = 1;
color_map = summer(size(win_locs,1));

figure(2)
% First Column
subplot(srows, scols, 1);
a1 = plot(tang, feat.angles.angles(I,2),'b','LineWidth', 2); hold on; 
a2 = plot(tang, feat.angles.angles(I,3), 'm','LineWidth', 2); grid on; 
ylim([0 180]);
xlim([tmin, tmax]);
box off;
for i = 1:size(win_locs,1)
    a = area(win_locs(i,:), [180 180], 'FaceColor', color_map(i,:));
    a.FaceAlpha = 0.2;
    a.EdgeAlpha = 0.2;
    a.BaseLine.Visible = false;
    
    offset = 0.1;
    if i==4
       offset = -0.02;
    end
        
    text(mean(win_locs(i,:))-offset, 25, ...
    ['\lambda=', num2str(round(lfcons(i),2))],'FontSize', 15);

end
ylabel('angle [deg]', 'FontSize', 16)
legend({'\alpha_1', '\alpha_2'}, 'NumColumns', 2)
xticks([])
yticks([0,180])
hold off;


subplot(srows, scols, 2);
plot(tang, fstr(:,1), 'b','LineWidth', 2); grid on; hold on;
ymax = max(20,max(abs(fstr(:))+0.4));
ylim([-ymax, 13]);
xlim([tmin, tmax]);
for i = 1:size(win_locs,1)
    a = area(win_locs(i,:), [ymax ymax], -ymax, 'FaceColor', color_map(i,:));
    a.FaceAlpha = 0.2;
    a.EdgeAlpha = 0.2;
    a.BaseLine.Visible = false;
    text(mean(win_locs(i,:))-0.05, -11, cluster_names{seq(i)}, 'FontWeight', 'bold', 'FontSize', 16);

end
legend('F_{str}')
ylabel('Force [N]', 'FontSize', 16)
box off;
xticks([])
hold off;


subplot(srows, scols, 3);
plot(tang, obs.pose123.linvel(I,1), 'b', tang, obs.pose123.linvel(I,2), 'm','LineWidth', 2); 
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
ylabel('velocity [m/s]', 'FontSize', 16)
box off;
xlabel('time [sec]', 'FontSize', 16)
legend({'v_x', 'v_y'}, 'Location', 'SouthEast', 'NumColumns', 2)
xticks(unique(win_locs(:))')





hold off;


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

%% Heatmaps and plots:  Primitive Clusters



tb_temp = primtable(primtable.obs_ind ~= 86 & primtable.obs_ind~=102, : );

% ks = unique(tb_temp.cluster);
ks = [2, 1, 6, 3, 4, 5];
% ks = unique(idx_fv);

ncols = 3;
nrows = 4;

figure(300);
for j = 1:numel(ks)
    
    sl = tb_temp(tb_temp.cluster==ks(j),:);
    
    [hist_mat_f,hist_mat_v, fstr_label, vel_label, tlabel] ...
        = signal_histogram(sl);
    
    hist_mat_f = hist_mat_f/110;
    hist_mat_v = hist_mat_v/110;
    
    
    snum_f = 3*floor(j/3.1) + j;
    subplot(nrows, ncols, snum_f);
    h = heatmap(hist_mat_f, 'Colormap', jet, 'ColorLimits', [0, 0.17]);
    title(cluster_names{ks(j)})
    h.XDisplayLabels = tlabel;
    h.YDisplayLabels = fstr_label;
    h.Position(3:4) = [0.2, 0.2];
    h.FontSize = 12;
    
    
    if mod(3*floor(j/3.1) + j,6) == 1
        ylabel('F_{str} [N]')
    end
    
    
    
    snum_v = 3*floor(j/3.1) + 3 + j;
    subplot(nrows, ncols, snum_v);
    h = heatmap(hist_mat_v, 'Colormap', jet, 'ColorLimits', [0, 0.17]);
    h.XDisplayLabels = tlabel;
    h.YDisplayLabels = vel_label;
    h.Position(3:4) = [0.2, 0.2];
    h.FontSize = 13;
    
    
    if mod(3*floor(j/3.1) + 3 + j, 6) == 4
        ylabel('v_{y} [m/s]')
    end
    
    if 2*j>=8
        xlabel('Time [sec]')
    end
    
end
% sgtitle('Primitive Clusters');





figure(401);

for j = 1:numel(ks)
    
    sl = tb_temp(tb_temp.cluster==ks(j),:);

    subplot(nrows,ncols, 3*floor(j/3.1) + j);
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
    ylabel('F_{str}');
    
    ylim([-20, 40])
    hold off; grid on;
%     ylabel('F_{str}')
    subtitle('F_{str}');
    
    subplot(nrows,ncols, 3*floor(j/3.1) + 3 + j);
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
    ylabel('v_y')
    ylim([-0.7, 0.7])
    grid on;
    subtitle(sprintf('cluster: %d #points: %d', ks(j), size(sl,1)));
    hold off;
    
end

sgtitle('Clusters');



% sprintf('cluster: %d #points: %d', ks(j), size(sl,1))

%% LF Consistency Histogram


figure(10);
histogram(primtable.lfcons,5);
ylabel('counts', 'FontSize', 15);
xlabel('\lambda', 'FontSize', 15);
xticks([1:0.2:2]);
yticks([0:40:120]);


%% Count the change of annotation of event detection

[~, primtable2, primNum2] = get_primitives(observations_processed, DetectedEvents_original);
[~, primtable, primNum] = get_primitives(observations_processed, DetectedEvents);


numSame = 0;

for obs_ind = 1:112
    sl1 = primtable(primtable.obs_ind==obs_ind,:);
    ev1 = [sl1.t_start, sl1.t_start];
    
    sl2 = primtable2(primtable2.obs_ind==obs_ind,:);
    ev2 = [sl2.t_start, sl2.t_start];
    
%     if and(ev1(:)==ev2(:))
%         numSame = numSame + size(ev1,1);
%     end
end












