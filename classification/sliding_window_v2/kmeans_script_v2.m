
clc;clear;close all;
base_path = '../../data/preprocessed_v2_2';

[observations_processed,tb] = load_data(base_path);

%% Customization
feature_inds = [1,4, 6, 9, 12, 15, 18, 28];
feature_names = {'FT1', 'FT2', 'F_{str}', 'F_{sum}', 'pose', 'vel', '\alpha', 'F_{str}^{integ}'};
label_names = {'obs_id', 'starting_table', 'window_num', 'traj_type',...
    'motion_type', 'initialOrientation', 'outcomeSubject'};
label_maps = containers.Map(label_names, 1:numel(label_names));

data_version = {'body_frame', 'spatial_frame', 'body_n_haptics',...
    'spatial_n_haptics', 'haptics', 'body_frame_td', 'body_frame_td_tnorm'};
norm_option = {'unnormalized','normalized'};
%% Load Features

best_pca_components = [20, 20, 20, 20, 10, 15, 10];

data_option = 7;

[X,Y] = extractSWFeatures(observations_processed, tb, data_option);
Nwinds = max(Y(:,3));

% Normalize
Xnorm = 2*(X-min(X))./(max(X)-min(X))-1;

% PCA
[coeff_norm, score_norm, latent_norm] = pca(Xnorm);

Xnorm_pca = Xnorm*coeff_norm;

ncomponents = best_pca_components(data_option);


[N, numFeats] = size(Xnorm_pca);




%% Kmeans
rng(2);

k = 3;

[idx_norm_pca,C_norm_pca,sumd_norm,D_norm] = kmeans(Xnorm_pca(:,1:ncomponents),k);


%% 
idx_temp = idx_norm_pca;
Xtemp = Xnorm_pca;

%% Visualize over in time domain

% close all

figure(1);
histogram2(Y(:,3), idx_temp, [max(Y(:,3)),k])
title('All Data')

figure(2);
I = Y(:,4) == 1;
% subplot(1,2,1);
histogram2(Y(I,3), idx_temp(I), [max(Y(:,3)),k])
title('Trajectory type AB1')
xlabel('time')
ylabel('clusters')
zlabel('occurance')

figure(3)
I = Y(:,4) == 2;
% subplot(1,2,2);
histogram2(Y(I,3), idx_temp(I), [max(Y(:,3)),k])
title('Trajectory type AB2')
xlabel('time')
ylabel('clusters')
zlabel('occurance')



%% LDA 


[Z1, W, lambda] = LDA(Xtemp, idx_temp);

figure(1);

for indk = 1:k
    scatter3(Z1(idx_temp==indk,1),Z1(idx_temp==indk,2),Z1(idx_temp==indk,3)); hold on;
end
hold off; grid on;



%% 3D plot in PCA space

idx_temp = idx_norm_pca;
Xtemp = Xnorm_pca;
figure(13);
cmap = hsv(k);

obs_id = 10;
c = zeros(numel(idx_temp),3);
for i =  1:k%numel(idx_temp)
    c(i,:) = cmap(idx_temp(i),:);
    Is = idx_temp==i;% & Y(:,1) == obs_id;
    scatter3(Xtemp(Is,1), Xtemp(Is,2), Xtemp(Is,3),10,cmap(i,:)); hold on;
end

xlabel('Component 1')
ylabel('Component 2')
zlabel('Component 3')

grid on; hold off;
legend(num2str([1:k]'));



%% Per observation

obs_id = 100;
obs = observations_processed(obs_id);

I = Y(:,1) == obs_id;

figure;
gscatter(Y(I,3), idx_norm_pca(I));
xlabel('Window number');
ylabel('cluster group')
title(sprintf('%s %s %s', obs.motion_type, obs.traj_type, obs.obs_id),...
    'Interpreter', 'none')
ylim([1,k])


%% PCA Manual cluster

