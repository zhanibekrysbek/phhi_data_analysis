
clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);


%% Load Features

[X,Y] = load_slidwind(observations_processed);



%% GMModel

% [coeff,score,latent] = pca(X);
% X = X*coeff;
rng(1);
k = 6;
gmfit = fitgmdist(X, k, 'RegularizationValue', 0.01);
idx = cluster(gmfit,X);


%% Visualize
close all

figure(1);
histogram2(Y(:,3)/max(Y(:,3)), idx, [max(Y(:,3)),k])
title('All Data')

figure(2);
I = Y(:,4) == 0;
% subplot(1,2,1);
histogram2(Y(I,3)/max(Y(I,3)), idx(I), [max(Y(:,3)),k])
title('Trajectory type AB1')

figure(3)
I = Y(:,4) == 1;
% subplot(1,2,2);
histogram2(Y(I,3)/max(Y(I,3)), idx(I), [max(Y(:,3)),k])
title('Trajectory type AB2')

figure(4)
I = Y(:,2) == 0;
% subplot(1,2,2);
histogram2(Y(I,3)/max(Y(I,3)), idx(I), [max(Y(:,3)),k])
title('Started from Table A')



%% LDA 


[Z1, W, lambda] = LDA(X, idx);

figure(1);

for indk = 1:k
    scatter3(Z1(idx==indk,1),Z1(idx==indk,2),Z1(idx==indk,3)); hold on;
end
hold off; grid on;



%% PCA

[coeff,score,latent] = pca(X);
X_ = X*coeff;

figure(2);
% hold on;
cmap = hsv(k);
c = zeros(numel(idx),3);
for i = 1:numel(idx)
    c(i,:) = cmap(idx(i),:);
end
scatter3(X_(:,1),X_(:,2),X_(:,3),10,c); 

% hold off; 
grid on;





