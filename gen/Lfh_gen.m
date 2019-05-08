function Lfh = Lfh_gen(in1)
%LFH_GEN
%    LFH = LFH_GEN(IN1)

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
t6 = 1.0./t4.^(3.0./2.0);
t7 = dx.*t5.*x;
t8 = dy.*t5.*y;
t9 = t7+t8;
Lfh = dx.*(t9.*(-dx.*t5+dx.*t2.*t6+dy.*t6.*x.*y)-t5.*x)+dy.*(t9.*(-dy.*t5+dy.*t3.*t6+dx.*t6.*x.*y)-t5.*y);
