function X1 = RobotDyn(X0, dT, V, a)

X1=zeros(3,1);
L=2.8;

X1(1,1)=X0(1,1)+dT*V*cos(X0(3,1));
X1(2,1)=X0(2,1)+dT*V*sin(X0(3,1));
X1(3,1)=X0(3,1)+dT*V*tan(a)/L;
% X1(4,1)=X0(4,1);
% X1(5,1)=X0(5,1);
% X1(6,1)=X0(6,1);
% X1(7,1)=X0(7,1);
% X1(8,1)=X0(8,1);
% X1(9,1)=X0(9,1);

end
