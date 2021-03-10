function primitives = normalize_primitives(primitives, btable)

apply_norm = @(sig, smax, smin) 2*(sig-smin)./(smax-smin)-1;


for ind = progress(1:numel(primitives), 'Title', 'NormalizingPrims')
    
    primitives(ind).prim.angles = apply_norm(primitives(ind).prim.angles, btable.angs_max, btable.angs_min);
    primitives(ind).prim.fstr = apply_norm(primitives(ind).prim.fstr, btable.fstr_max, btable.fstr_min);
    primitives(ind).prim.fsum = apply_norm(primitives(ind).prim.fsum, btable.fsum_max, btable.fsum_min);
    primitives(ind).prim.f1 = apply_norm(primitives(ind).prim.f1, btable.f1_max, btable.f1_min);
    primitives(ind).prim.f2 = apply_norm(primitives(ind).prim.f2, btable.f2_max, btable.f2_min);
    primitives(ind).prim.tau1 = apply_norm(primitives(ind).prim.tau1, btable.tau1_max, btable.tau1_min);
    primitives(ind).prim.tau2 = apply_norm(primitives(ind).prim.tau2, btable.tau2_max, btable.tau2_min);
    primitives(ind).prim.vel = apply_norm(primitives(ind).prim.vel, btable.vel_max, btable.vel_min);
    primitives(ind).prim.angvel = apply_norm(primitives(ind).prim.angvel, btable.angvel_max, btable.angvel_min);
    primitives(ind).prim.pos = apply_norm(primitives(ind).prim.pos, btable.pos_max, btable.pos_min);
    primitives(ind).prim.orient = apply_norm(primitives(ind).prim.orient, btable.orient_max, btable.orient_min);

end

end


