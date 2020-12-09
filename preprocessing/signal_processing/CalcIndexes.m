function AllIndx = CalcIndexes(ForceData, model)
%  AllIndx = [Ic Id Ie If Is];

t=ForceData.T_z(1):ForceData.T_z(3);
switch model
    case 'PM'
        F1e = ForceData.F1e(t);
        F2e = ForceData.F2e(t);
        Delta=CalcDeltaPM(ForceData);
    case 'VL'
        F1e = ForceData.Fsum(t)/2;
        F2e = ForceData.Fsum(t)/2;
        Delta=zeros(size(F1e));
    case 'ME'
        Delta=CalcDeltaHu(ForceData);
        F1e = (.5*ones(size(Delta)) + Delta).*ForceData.Fsum(t);
        F2e = ForceData.Fsum(t) - F1e;

        F1e(Delta <=-.5) = 0;
        F2e(Delta >=0.5) = 0;
end

Ic = Coop(Delta);
If = Fairness(F1e, F2e);
Id = Difficulty(Delta);
Is = Similarity(Delta);
Ie = Efficiency(Delta);

AllIndx = [Ic Id Ie If Is];


function Delta=CalcDeltaHu(ForceData)

t=ForceData.T_z(1):ForceData.T_z(3);
Fsum= ForceData.Fsum(t);
nFs = mNorm(Fsum);
nFs(nFs<.05) = nan;

F1= ForceData.F1(t);

Delta = -.5 + sum(F1.*Fsum,2)./nFs;

I0 = find(isnan(Delta),1, 'first');
while ~isempty(I0)
I1 = find(~isnan(Delta(I0:end)),1, 'first');
if isempty(I1)
Delta(I0:end) = repmat(Delta(I0-1),length(Delta)-I0+1,1);
else
Delta(I0:I0+I1-1) = repmat(Delta(I1),I1,1);
end
I0 = find(isnan(Delta),1, 'first');
end

function Delta=CalcDeltaPM(ForceData)

t=ForceData.T_z(1):ForceData.T_z(3);
F1e = ForceData.F1e(t);
Fsum= ForceData.Fsum(t);
nFs = mNorm(Fsum);
nFs(nFs<.05) = nan; % 0.05 -  original threshold
Delta = mNorm(F1e - Fsum/2)./nFs;

I0 = find(isnan(Delta),1, 'first');
while ~isempty(I0)
    I1 = find(~isnan(Delta(I0:end)),1, 'first');
    if isempty(I1)
        Delta(I0:end) = repmat(Delta(I0-1),length(Delta)-I0+1,1);
    else
        Delta(I0:I0+I1-1) = repmat(Delta(I1),I1,1);
    end
    I0 = find(isnan(Delta),1, 'first');
end

function nV = mNorm(V)

nV = sqrt(sum(V.^2,2));

function Ic = Coop(Delta)

% Mc = ones(length(Delta),1)-2*abs(Delta);
Ic = 1- mean(Delta);

function Is = Similarity(Delta)

Ms = ones(length(Delta),1)-2*abs(Delta);
Ms(abs(Delta)>.5) = 0;
Is = mean(Ms);

function If = Fairness(F1e, F2e)

N1 = sum(mNorm(F1e));
N2 = sum(mNorm(F2e));
Ns = sum(mNorm(F1e + F2e));
If = 1 - abs((N1-N2)/Ns);

function Id = Difficulty(Delta)

dDelta = diff(Delta)/1e-3;
Id = mean(abs(dDelta));
Id = 1 - Id/40;

function Ie = Efficiency(Delta)

Me = ones(length(Delta),1)./(2*abs(Delta));
Me(abs(Delta)<=.5) = 1;
Ie = mean(Me);

