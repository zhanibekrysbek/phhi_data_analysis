
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
xnorm =@(X) 2*(X-min(X))./(max(X)-min(X))-1;
% Xnorm = 2*(X-min(X))./(max(X)-min(X))-1;
Xnorm = xnorm(X);

% PCA
[coeff,score,latent] = pca(X);
[coeff_norm, score_norm, latent_norm] = pca(Xnorm);

X_pca = X*coeff;
Xnorm_pca = Xnorm*coeff_norm;

%% Slicing

rng(11);
k = 2;


% Remove Lifting Up and Down components

% Separation line X(:,[2,3]) plane 
p1 = [1.238, -0.442];
p2 = [0.656, 2.158];
P = [p1;p2];
a = polyfit(P(:,1),P(:,2),1);

I1 = Xnorm_pca(:,3) - Xnorm_pca(:,2)*a(1) - a(2) > 0;

idx_norm_pca = zeros(size(X,1),1);
% idx_norm_pca(I1) = 1;
% idx_norm_pca(~I1) = 2;

% Focus on the body
X1 = X(I1,:);
Y1 = Y(I1,:);

% Separation line X(:,[1,3]) plane 
p1 = [-1.918, 0.75];
p2 = [1.94, 1.328];
P = [p1;p2];
a = polyfit(P(:,1),P(:,2),1);

I2 = Xnorm_pca(:,3) - Xnorm_pca(:,1)*a(1) - a(2) > 0;

idx_norm_pca = zeros(size(X1,1),1);
idx_norm_pca(I2) = 1;
idx_norm_pca(~I2) = 2;

% Normalize
% Focus on the body
% X1 = X(I1,:);
% Y1 = Y(I1,:);

% [coeff,score,latent] = pca(X1);
% [coeff_norm, score_norm, latent_norm] = pca(Xnorm_pca);

% X_pca = X*coeff;
% Xnorm_pca = Xnorm*coeff_norm;

% [idx_norm_pca,C_norm_pca,sumd_norm,D_norm] = kmeans(Xnorm_pca(:,1:20),k);

%% 
idx_temp = idx_norm_pca;
Xtemp = Xnorm_pca;

Ytemp = Y1;
%% Visualize over in time domain

% close all

figure(1);
histogram2(Ytemp(:,3)/max(Ytemp(:,3)), idx_temp, [max(Ytemp(:,3)),k])
title('All Data')

figure(2);
I = Ytemp(:,4) == 0;
% subplot(1,2,1);
histogram2(Y(I,3)/max(Y(I,3)), idx_temp(I), [max(Ytemp(:,3)),k])
title('Trajectory type AB1')
xlabel('time')
ylabel('clusters')
zlabel('occurance')

figure(3)
I = Ytemp(:,4) == 1;
% subplot(1,2,2);
histogram2(Ytemp(I,3)/max(Ytemp(I,3)), idx_temp(I), [max(Ytemp(:,3)),k])
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
I = Ytemp(:,5) == 0;
% subplot(1,2,2);
histogram2(Ytemp(I,3)/max(Ytemp(I,3)), idx_temp(I), [max(Ytemp(:,3)),k])
title('Serial Movement')
xlabel('time')
ylabel('clusters')
zlabel('occurance')

figure(6)
I = Ytemp(:,5) == 1;
% subplot(1,2,2);
histogram2(Ytemp(I,3)/max(Ytemp(I,3)), idx_temp(I), [max(Ytemp(:,3)),k])
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

c = zeros(numel(idx_temp),3);
for i = 1:k%numel(idx_temp)
%     c(i,:) = cmap(idx_temp(i),:);
    Is = idx_temp==i;
    scatter3(Xtemp(Is,1), Xtemp(Is,2), Xtemp(Is,3),10,cmap(i,:)); hold on;
end

xlabel('Component 1')
ylabel('Component 2')
zlabel('Component 3')

grid on; hold off;
legend(num2str([1:k]'));



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



