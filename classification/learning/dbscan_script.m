
clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);

%% Customization
feature_inds = [0,30, 60, 95, 125];
feature_names = {'rft1', 'rft2', 'pose', 'twist', 'accel'};

data_version = {'body_frame', 'spatial_frame'};
norm_option = {'unnormalized','normalized'};

%% Load Features

data_option = 1;

[X,Y] = extractSWFeatures(observations_processed, data_option);

% Normalize
Xnorm = 2*(X-min(X))./(max(X)-min(X))-1;

% PCA
[coeff,score,latent] = pca(X);
[coeff_norm, score_norm, latent_norm] = pca(Xnorm);

X_pca = X*coeff;
Xnorm_pca = Xnorm*coeff_norm;

%% DBSCAN

% Conclusion: Not suitable to this data. Fundamentally, it clusters by
% spatial distance among the points, and tends to suffer for higher
% dimensional data (more than 3).
rng(2);

minpts = 70;
epsilon = 1.5;


[idx,COREPTS] = dbscan(X,epsilon, minpts);
[idx_norm,COREPTS] = dbscan(Xnorm(:,1:3),epsilon, minpts);

[idx_pca,COREPTS] = dbscan(X_pca,epsilon, minpts);
[idx_norm_pca,COREPTS] = dbscan(Xnorm_pca(:,1:3),epsilon, minpts);


%% 
idx_temp = idx_norm_pca;
Xtemp = Xnorm_pca;
k = numel(unique(idx_temp))+1;

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
figure(3);
cmap = hsv(k);

c = zeros(numel(idx_temp),3);
for i = 1:numel(idx_temp)
    if idx_temp(i)==-1
        c(i,:) = cmap(idx_temp(i)+2,:);
    else
        c(i,:) = cmap(idx_temp(i)+1,:);
    end
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


%% PCA Manual cluster

% X_ = 
