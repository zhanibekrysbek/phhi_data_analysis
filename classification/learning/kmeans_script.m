
clc;clear;
base_path = '../../data/preprocessed_v2_1';

[observations_processed,tb] = load_data(base_path);


%% Load Features

[X,Y] = load_slidwind(observations_processed);



%% Kmeans
k = 3;

[idx,C,sumd,D] = kmeans(X,k);




%% Visualize
close all

% figure(1);
% 
% histogram(idx(Y(:,1)==1));


% figure(2);
% hold on;
% for i=1:112
% I = Y(:,1) == i;
% scatter(Y(I,2)/max(Y(I,2)), idx(I));
% end
% hold off;


figure(3);
I = Y(:,3) == 0;
% subplot(1,2,1);
histogram2(Y(I,2)/max(Y(I,2)), idx(I), [max(Y(:,2)),k])

figure(4)
I = Y(:,3) == 1;
% subplot(1,2,2);
histogram2(Y(I,2)/max(Y(I,2)), idx(I), [max(Y(:,2)),k])



%% LDA 


[Z1, W, lambda] = LDA(X, idx);

figure(1);

for indk = 1:k
    scatter3(Z1(idx==indk,1),Z1(idx==indk,2),Z1(idx==indk,3)); hold on;
end
hold off; grid on;



% PCA
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





