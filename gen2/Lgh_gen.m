function Lgh = Lgh_gen(in1,in2)
%LGH_GEN
%    LGH = LGH_GEN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.2.
%    07-May-2019 19:53:37

theta = in1(4,:);
u1 = in2(1,:);
u2 = in2(2,:);
vel = in1(3,:);
x = in1(1,:);
y = in1(2,:);
t2 = x.^2;
t3 = y.^2;
t4 = t2+t3+1.0e-5;
t5 = 1.0./sqrt(t4);
t6 = cos(theta);
t7 = sin(theta);
t8 = t5.*t6.*vel.*x;
t9 = t5.*t7.*vel.*y;
t10 = t8+t9;
Lgh = t10.*u2.*(t5.*t7.*vel.*x-t5.*t6.*vel.*y)-t10.*u1.*(t5.*t6.*x+t5.*t7.*y);