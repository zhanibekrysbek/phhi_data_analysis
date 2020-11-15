
clc;clear;
%% Load the data


base_path = '../../data/preprocessed_v1';

files = dir(base_path);

observations(numel(files)-2) = struct();

for i=1:numel(files)
    f = files(i);
    if ~f.isdir
        ld = load(fullfile(f.folder,f.name));
        fns = fieldnames(ld);
        for fn=1:numel(fns)
            observations(i-2).(fns{fn}) = ld.(fns{fn});
        end
    end
end

%% Preprocess RFT and Pose

rftFS = 1000;
imuFS = 100;
cutoff = 12.5;

Hd = designfilt('lowpassfir','FilterOrder',100,'CutoffFrequency',cutoff, ...
       'DesignMethod','window','Window',{@kaiser,3},'SampleRate',rftFS);
   
rft_ids = {'C00300119','C00300122'};

observations_processed = observations;
method = 'pchip';

for i=1:numel(observations_processed)
    
    [observations_processed(i),tf] = process_rft(observations(i));
    
    % Outlier removal
    obs = observations_processed(i);

    method = 'median';
%     [obs.pose123.position,I] = rmoutliers(obs.pose123.position,method);
    
    obs.pose123.orientation(:,4) = unwrap(obs.pose123.orientation(:,4),1.98*pi);
    [obs.pose123.orientation,I] = rmoutliers(obs.pose123.orientation,method);

    obs.pose123.time_steps = obs.pose123.time_steps(~I);
    obs.pose123.position = obs.pose123.position(~I,:);
    observations_processed(i) = obs;
end


%% Plot a dozen of samples
for i=1:10
    obs = observations_processed(i);
    obs2 = observations(i);
    
    figure(i+1);
    
    subplot(221)
    plot(obs.pose123.time_steps, obs.pose123.position,'.')
    % plot(obs.pose123.position,'.')
    grid on;
    subtitle('rmoutlier Position')
    ylabel('m')
%     legend('x','y','z')

    subplot(223)
    % plot(obs.pose123.orientation,'.')
    plot(obs.pose123.time_steps, obs.pose123.orientation,'.')

    grid on;
    subtitle('Orientation')
    xlabel('time, s')
    ylabel('axis-angle')
%     legend('x','y','z','angle')
    
    
    subplot(222)
    plot(obs2.pose123.time_steps, obs2.pose123.position,'.')
    % plot(obs.pose123.position,'.')
    grid on;
    subtitle('Raw Position')
    ylabel('m')
    legend('x','y','z')

    subplot(224)
    % plot(obs.pose123.orientation,'.')
    plot(obs2.pose123.time_steps, obs2.pose123.orientation,'.')

    grid on;
    subtitle('Orientation')
    xlabel('time, s')
    ylabel('axis-angle')
%     legend('x','y','z','angle')

    sgtitle(sprintf('%s %s %s Position', obs.obs_id, obs.traj_type, obs.motion_type ), ...
            'Interpreter','none');
    
end


%% Draft for pose
ind = 50;
obs = observations(ind);

method = 'median';
% [obs.pose123.position,I] = rmoutliers(obs.pose123.position,method);

[obs.pose123.orientation,I] = rmoutliers(obs.pose123.orientation,method);

obs.pose123.time_steps = obs.pose123.time_steps(~I);
obs.pose123.position = obs.pose123.position(~I,:);


figure(50)
plot_pose(obs)

figure(51)
plot_pose(observations(ind))



%% Plot Sample
figure(1);
ind = randi(numel(observations))
plot_rfts(observations_processed(ind))




