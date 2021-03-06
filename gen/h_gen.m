function h = h_gen(in1)
%H_GEN
%    H = H_GEN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    07-May-2019 17:56:02

dx = in1(3,:);
dy = in1(4,:);
x = in1(1,:);
y = in1(2,:);
t2 = x.^2;
t3 = y.^2;
t4 = t2+t3;
t5 = 1.0./sqrt(t4);
t6 = dx.*t5.*x+dy.*t5.*y;
h = -sqrt(t4)-t6.^2./2.0+1.0;
