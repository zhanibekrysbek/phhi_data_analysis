
clc;clear;close all;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);

%% Customization
feature_inds = [0,30, 60, 95, 125];
feature_names = {'rft1', 'rft2', 'pose', 'twist', 'accel'};

data_version = {'body_frame', 'spatial_frame', 'body_n_haptics', 'spatial_n_haptics', 'haptics'};
norm_option = {'unnormalized','normalized'};

%% Load Features

best_pca_components = [20, 20, 20, 20, 20];

data_option = 1;

[X,Y] = extractSWFeatures(observations_processed, data_option);
Nwinds = max(Y(:,3));

% Normalize
Xnorm = 2*(X-min(X))./(max(X)-min(X))-1;

% PCA
[coeff_norm, score_norm, latent_norm] = pca(Xnorm);

Xnorm_pca = Xnorm*coeff_norm;

ncomponents = best_pca_components(data_option);

% First p percent of interaction
Iint = Y(:,3)/Nwinds < 0.6;
Xnorm_pca = Xnorm_pca(Iint,:);
Y = Y(Iint,:);

[N, numFeats] = size(Xnorm_pca);

%% Kmeans
rng(2);

k = 7;

[idx_norm_pca,C_norm_pca,sumd_norm,D_norm] = kmeans(Xnorm_pca(:,1:ncomponents),k);


%% 
idx_temp = idx_norm_pca;
Xtemp = Xnorm_pca;

%% Visualize over in time domain

% close all

figure(1);
histogram2(Y(:,3)/Nwinds, idx_temp, [max(Y(:,3)),k])
title('All Data')

figure(2);
I = Y(:,4) == 0;
% subplot(1,2,1);
histogram2(Y(I,3)/Nwinds, idx_temp(I), [max(Y(:,3)),k])
title('Trajectory type AB1')
xlabel('time')
ylabel('clusters')
zlabel('occurance')

figure(3)
I = Y(:,4) == 1;
% subplot(1,2,2);
histogram2(Y(I,3)/Nwinds, idx_temp(I), [max(Y(:,3)),k])
title('Trajectory type AB2')
xlabel('time')
ylabel('clusters')
zlabel('occurance')

% figure(4)
% I = Y(:,2) == 0;
% % subplot(1,2,2);
% histogram2(Y(I,3)/max(Y(I,3)), idx_temp(I), [max(Y(:,3)),k])
% title('Started from Table A')


figure(5)
I = Y(:,5) == 0;
% subplot(1,2,2);
histogram2(Y(I,3)/Nwinds, idx_temp(I), [max(Y(:,3)),k])
title('Serial Movement')
xlabel('time')
ylabel('clusters')
zlabel('occurance')

figure(6)
I = Y(:,5) == 1;
% subplot(1,2,2);
histogram2(Y(I,3)/Nwinds, idx_temp(I), [max(Y(:,3)),k])
title('Parallel Movement')
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

obs_id = 100;
c = zeros(numel(idx_temp),3);
for i =  1:k%numel(idx_temp)
    c(i,:) = cmap(idx_temp(i),:);
    Is = idx_temp==i;
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
gscatter(Y(I,3)/Nwinds, idx_norm_pca(I));
xlabel('normalized time');
ylabel('cluster group')
title(sprintf('%s %s %s', obs.motion_type, obs.traj_type, obs.obs_id),...
    'Interpreter', 'none')
ylim([1,k])


%% PCA Manual cluster

% X_ = 
