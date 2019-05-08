function cons = cbf_constraint_unicycle(x, u)

    if h_gen(x) < 0
        cons = x(3)^2 + u(1)^2;
    else
        cons = Lfh_gen(x) + Lgh_gen(x, u) + h_gen(x);
    end
end

