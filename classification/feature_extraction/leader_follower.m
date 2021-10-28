
clc;clear;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);

load('../../data/events/DetectedEvents_v2.mat');

[features] = haptic_features(observations_processed);

st = struct2table(DetectedEvents);
st = convertvars(st, {'obs_id'}, 'string');
M = containers.Map(st.obs_id, st.events);




%% Event Summarization


% Count total number of events
N = 0;
for i = 1:numel(DetectedEvents)
   N = N + size(DetectedEvents(i).events,1);
end

eventsTable = array2table(zeros(N, 12), 'VariableNames', {'obs_ind', 'seqnum', 'lfcons',...
    'schange', 'fstr_extr', 'fstr_mean', 'fstr_t_extr', 'exp_dir', 'outcome', 'vy_gain', 'vy_mean', 'inst_dec'});
ev_ind = 0;

for obs_ind = progress(1:numel(DetectedEvents), 'Title', 'SummarizingEvents')
    obs = observations_processed(obs_ind);
    feat = features(obs_ind);
    
    ev_mat = M(obs.obs_id);
    for seqnum = 1:size(ev_mat,1)
        
        ev = ev_mat(seqnum,:);
        
        % Time frame
        Ipos = obs.pose123.time_steps >= ev(1) & obs.pose123.time_steps <= ev(2);
        Irft = obs.rft1.time_steps >= ev(1) & obs.rft1.time_steps <= ev(2);
        t_ev = obs.rft1.time_steps(Irft);
        
        % Get signals
        ang = feat.angles.angles(Ipos, 2) - feat.angles.angles(Ipos, 3);
        fstr = obs.fstretch.force(Irft, 1);
        vy = obs.pose123.linvel(Ipos, 2);

        % Variables of interest
        lfcons = sum(ang>0)/numel(ang) + 1.; % closer to 1 - F1 is in leader

        pos = ang>0;
        changes = xor(pos(1:end-1),pos(2:end));
        schange = sum(changes);

        [a,b] = max(abs(fstr));
        fstr_extr = fstr(b);
        fstr_mean = mean(fstr);
        fstr_t_extr = t_ev(b) - t_ev(1);

        vy_gain = vy(end) - vy(1);
        vy_mean = mean(vy);

        exp_dir = 0; % Neutral
        if abs(vy(end)) >= 0.05
            % vy > 0 - left
            % exp_dir= -1 - left
            % exp_dir= 1 - right 
            exp_dir = -sign(vy(end)); 
        end
        
        outcome = 1;
        if strcmp(obs.traj_type, 'AB1') % left
            outcome = -1;
        end
        
        ev_ind = ev_ind+1;
        eventsTable(ev_ind,:) = num2cell([obs_ind, seqnum, lfcons, schange, ...
            fstr_extr, fstr_mean, fstr_t_extr, exp_dir, outcome, vy_gain, ...
            vy_mean, double(obs.inst_dec)]); 
    end
end

%% EventsTable plots

figure(21);
scatter(eventsTable.fstr_mean, eventsTable.lfcons, '.'); grid on;
title('F_{str} vs LF consistency')

figure(22);
histogram(eventsTable.lfcons, 5)
title('Histogram of LF consistency')



%% Plot Angles

obs_ind = 49;

feat = features(obs_ind);
obs = observations_processed(obs_ind);

win_locs = M(obs.obs_id);
        


ang_inds = [1, 2];
srows = 5;

I = feat.angles.time_steps <= obs.tdec_sec;
tang = feat.angles.time_steps(I);
fstr = obs.fstretch.force(1:10:end,1);
fstr = fstr(I);
orient = rad2deg(obs.imu.orientation(I,4));
orient = orient - orient(1);

figure(obs_ind);

subplot(srows, 1, 1);
plot(tang, feat.angles.angles(I,1)); grid on;
subtitle('F_1 F_2 angle')
xlim([0, max(5,obs.tdec_sec)])



subplot(srows, 1, 2);
plot(tang, feat.angles.angles(I,2), 'DisplayName', 'f_1 vs v');hold on; 
plot(tang, feat.angles.angles(I,3), 'DisplayName','f_2 vs v'); grid on; 
legend;

for i = 1:size(win_locs,1)
    xline(win_locs(i,1),'-.b', 't0', 'DisplayName', '');
    xline(win_locs(i,2),'-.b', 'tf','LabelVerticalAlignment','bottom', 'DisplayName', '');
end
hold off;
xlim([0, max(5,obs.tdec_sec)])
subtitle('F_1 vs v and F_2 vs v angles');



subplot(srows, 1, 3);
plot(tang, fstr); grid on; hold on;
subtitle('F_{str}');

for i = 1:size(win_locs,1)
    xline(win_locs(i,1),'-.b', 't0');
    xline(win_locs(i,2),'-.b', 'tf','LabelVerticalAlignment','bottom');
end
hold off;
ymax = max(10,max(abs(fstr)));
ylim([-ymax, ymax]);
xlim([0, max(5,obs.tdec_sec)])


subplot(srows,1,4)
plot(tang, obs.pose123.linvel(I,1), tang, obs.pose123.linvel(I,2)); grid on;
subtitle('v_x v_y')
xlim([0, max(5,round(obs.tdec_sec))])
ylim([-0.8 0.8])

for i = 1:size(win_locs,1)
    xline(win_locs(i,1),'-.b', 't0');
    xline(win_locs(i,2),'-.b', 'tf','LabelVerticalAlignment','bottom');
end

subplot(srows,1,5)
plot(tang, orient); grid on;
subtitle('orientation angle')
xlim([0, max(5,round(obs.tdec_sec))])
for i = 1:size(win_locs,1)
    xline(win_locs(i,1),'-.b', 't0');
    xline(win_locs(i,2),'-.b', 'tf','LabelVerticalAlignment','bottom');
end
xlabel('time, sec')

sgtitle(sprintf('%s', obs.obs_id), 'Interpreter', 'None')


figure(2);
plot_rfts(obs,1)



