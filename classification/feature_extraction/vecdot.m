function [dotsv] = vecdot(v1,v2)
%vecdot Summary of this function goes here
%   Dot product for 3 dim vectors stacked vertically. v1 and v2 must be of
% dimension Nx2 or Nx3.

dotsv = sum(v1.*v2,2);

end

