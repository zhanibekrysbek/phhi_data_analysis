function [dist] = prim_distance(prim1, prim2)



fdist = get_force_dist(prim1,prim2);
    
kdist = get_kin_dist(prim1,prim2);

angdist = get_ang_dist(prim1,prim2);

dist = [fdist kdist angdist];

end


function fdist = get_force_dist(prim1, prim2)
    
fdist = zeros(1,4);

fdist(1) = dtw(prim1.fstr(:,1), prim2.fstr(:,1)) + dtw(prim1.fstr(:,2), prim2.fstr(:,2));

fdist(2) = dtw(prim1.fsum(:,1), prim2.fsum(:,1)) + dtw(prim1.fsum(:,2), prim2.fsum(:,2));

f11 = dtw(prim1.f1(:,1), prim2.f1(:,1)) + dtw(prim1.f1(:,2), prim2.f1(:,2))...
    + dtw(prim1.f2(:,1) , prim2.f2(:,1)) + dtw(prim1.f2(:,2) , prim2.f2(:,2));
f12 = dtw(prim1.f1(:,1), prim2.f2(:,1)) + dtw(prim1.f2(:,2), prim2.f1(:,2))...
    + dtw(prim1.f1(:,1), prim2.f2(:,1)) + dtw(prim1.f2(:,2), prim2.f1(:,2));
fdist(3) = min(f11, f12);
        
fdist(4) = min(dtw(prim1.tau1, prim2.tau1) + dtw(prim1.tau2, prim2.tau2), ...
    dtw(prim1.tau1, prim2.tau2)+dtw(prim1.tau2, prim2.tau1));
    
end


function kdist = get_kin_dist(prim1,prim2)

kdist = zeros(1,4);

kdist(1) = dtw(prim1.pos(:,1),prim2.pos(:,1)) + dtw(prim1.pos(:,2),prim2.pos(:,2));
kdist(2) = dtw(prim1.orient, prim2.orient);
kdist(3) = dtw(prim1.vel(:,1), prim2.vel(:,1)) + dtw(prim1.vel(:,2), prim2.vel(:,2));
kdist(4) = dtw(prim1.angvel, prim2.angvel);

end



function angdist = get_ang_dist(prim1,prim2)

angdist = zeros(1,2);

angs1 = deg2rad(prim1.angles);
angs2 = deg2rad(prim2.angles);

angs1(isnan(angs1)) = 0;
angs2(isnan(angs2)) = 0;

angdist(1) = dtw(angs1(:,1), angs2(:,1));

a11 = dtw(angs1(:,2), angs2(:,2)) + dtw(angs1(:,3), angs2(:,3));
a12 = dtw(angs1(:,2), angs2(:,3)) + dtw(angs1(:,3), angs2(:,2));

angdist(2) = min(a11,a12);


end