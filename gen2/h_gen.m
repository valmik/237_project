function h = h_gen(in1)
%H_GEN
%    H = H_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    07-May-2019 19:53:37

theta = in1(4,:);
vel = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = x.^2;
t3 = y.^2;
t4 = t2+t3+1.0e-5;
t5 = 1.0./sqrt(t4);
t6 = t5.*vel.*x.*cos(theta)+t5.*vel.*y.*sin(theta);
h = -sqrt(t4)-t6.^2./2.0+1.0;