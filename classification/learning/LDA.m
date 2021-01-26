%
% Copyright (c) 2015, Yarpiz (www.yarpiz.com)
% All rights reserved. Please read the "license.txt" for license terms.
%
% Project Code: YPML114
% Project Title: Implementation of Linear Discriminant Analysis in MATLAB
% Publisher: Yarpiz (www.yarpiz.com)
% 
% Developer: S. Mostapha Kalami Heris (Member of Yarpiz Team)
% 
% Contact Info: sm.kalami@gmail.com, info@yarpiz.com
%

% function [Y, W, lambda] = LDA(X, L)
% 
%     Classes=unique(L)';
%     k=numel(Classes);
%     
%     n=zeros(k,1);
%     C=cell(k,1);
%     M=mean(X);
%     S=cell(k,1);
%     Sw=0;
%     Sb=0;
%     for j=1:k
%         Xj=X(L==Classes(j),:);
%         n(j)=size(Xj,1);
%         C{j}=mean(Xj);
%         S{j}=0;
%         for i=1:n(j)
%             S{j}=S{j}+(Xj(i,:)-C{j})'*(Xj(i,:)-C{j});
%         end
%         Sw=Sw+S{j};
%         Sb=Sb+n(j)*(C{j}-M)'*(C{j}-M);
%     end
%     [W, LAMBDA]=eig(Sb,Sw);
%     lambda=diag(LAMBDA);
%     [lambda, SortOrder]=sort(lambda,'descend');
%     W=W(:,SortOrder);
%     Y=X*W;
% end




function [Y, W, lambda] = LDA(X, L)

    Classes=unique(L)';
    k=numel(Classes);
    
    NumFeatures = size(X,2);
    n=zeros(k,1);
    C=cell(k,1);
    M=mean(X);
    S=cell(k,1);
    Sw=0;
    Sb=0;
    for j=1:k
        Xj=X(L==Classes(j),:);
        n(j)=size(Xj,1);
        C{j}=mean(Xj);
        C_rep = repmat(C{j},n(j),1);
        XC = Xj-C_rep;
        S{j}=0;
        for k1=1:NumFeatures
            for k2=k1:NumFeatures
                S{j}(k1,k2) = XC(:,k1)'*XC(:,k2);
                S{j}(k2,k1) = S{j}(k1,k2);
            end
        end
        Sw=Sw+S{j};
        Sb=Sb+n(j)*(C{j}-M)'*(C{j}-M);
    end

    [W, LAMBDA]=eig(Sb,Sw);

    lambda=diag(LAMBDA);

    [lambda, SortOrder]=sort(lambda,'descend');

    W=W(:,SortOrder);

    Y=X*W;

end