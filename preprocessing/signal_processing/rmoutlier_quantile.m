function [out, I] = rmoutlier_quantile(y, m)
%rmoutlier_quantile - outlier removal by box plot. 
% Bounds are determined m*iqr away from 25th and 75th quantiles.
%
%
%
%

qs = quantile(y, [0.25, 0.75]);
iqr = qs(2,:) - qs(1,:);
lb = qs(1,:)-m.*iqr;
ub = qs(2,:)+m.*iqr;
I = sum(y < lb,2) | sum(y>ub, 2);
out = y(~I,:);


end