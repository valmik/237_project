function cons = cbf_constraint(x, u)
    cons = Lfh_gen(x) + Lgh_gen(x, u) + h_gen(x);
end

