

clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);

%% Customization
feature_inds = [0,30, 60, 95, 125];
feature_names = {'rft1', 'rft2', 'pose', 'twist', 'accel'};

data_version = {'body_frame', 'spatial_frame'};
norm_option = {'unnormalized','normalized'};

%% Load Features

data_option = 2;

[X,Y] = extractSWFeatures(observations_processed, data_option);

% Normalize
Xnorm = 2*(X-min(X))./(max(X)-min(X))-1;

% PCA
[coeff,score,latent] = pca(X);
[coeff_norm, score_norm, latent_norm] = pca(Xnorm);

X_pca = X*coeff;
Xnorm_pca = Xnorm*coeff_norm;

%% Linkage

rng(2);
k = 6;

Z = linkage(Xnorm_pca);
idx_norm_pca = cluster(Z,'maxclust',6);

cutoff = median([Z(end-2,3) Z(end-1,3)]);
dendrogram(Z,'ColorThreshold',cutoff)

%% 
idx_temp = idx_norm_pca;
Xtemp = Xnorm_pca;

%% Visualize over in time domain
close all


figure(1);
histogram2(Y(:,3)/max(Y(:,3)), idx_temp, [max(Y(:,3)),k])
title('All Data')

figure(2);
I = Y(:,4) == 0;
% subplot(1,2,1);
histogram2(Y(I,3)/max(Y(I,3)), idx_temp(I), [max(Y(:,3)),k])
title('Trajectory type AB1')

figure(3)
I = Y(:,4) == 1;
% subplot(1,2,2);
histogram2(Y(I,3)/max(Y(I,3)), idx_temp(I), [max(Y(:,3)),k])
title('Trajectory type AB2')

% figure(4)
% I = Y(:,2) == 0;
% % subplot(1,2,2);
% histogram2(Y(I,3)/max(Y(I,3)), idx_temp(I), [max(Y(:,3)),k])
% title('Started from Table A')


figure(5)
I = Y(:,5) == 0;
% subplot(1,2,2);
histogram2(Y(I,3)/max(Y(I,3)), idx_temp(I), [max(Y(:,3)),k])
title('Serial Movement')

figure(6)
I = Y(:,5) == 1;
% subplot(1,2,2);
histogram2(Y(I,3)/max(Y(I,3)), idx_temp(I), [max(Y(:,3)),k])
title('Parallel Movement')


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
c = zeros(numel(idx_temp),3);
for i = 1:numel(idx_temp)
    c(i,:) = cmap(idx_temp(i),:);
end
scatter3(Xtemp(:,1), Xtemp(:,2), Xtemp(:,3),10,c); 
xlabel('Component 1')
ylabel('Component 2')
zlabel('Component 3')

grid on;
legend({'1', '2', '3'})



%% Per observation

obs_id = 1;
obs = observations_processed(obs_id);

I = Y(:,1) == obs_id;

figure;
gscatter(Y(I,3)/max(Y(I,3)), idx(I));
xlabel('normalized time');
ylabel('cluster group')
title(sprintf('%s %s %s', obs.motion_type, obs.traj_type, obs.obs_id),...
    'Interpreter', 'none')








