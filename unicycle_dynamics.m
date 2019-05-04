function [dq] = unicycle_dynamics(q, u, ts)
%UNICYCLE_DYNAMICS Forward euler discretization of the unicycle model
%   The state is q = [x; y; theta]. The input is u = [v; w]
%   ts is the discretization timestep

dq = q + [u(1)*cos(q(3)); u(1)*sin(q(3)); u(2)]*ts;

end

