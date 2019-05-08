function cons = cbf_constraint(x, u, ts)
%     if ~any(x)
%         cons = norm(u, inf);
%     else
    if x(1) == 0 && x(2) == 0
%         cons = Lfh2_gen(x, ts) + Lgh2_gen(x, u, ts) + h2_gen(x, ts);
        cons = abs(u)*Inf;
    else
        cons = Lfh_gen(x) + Lgh_gen(x, u) + h_gen(x);
    end
end

