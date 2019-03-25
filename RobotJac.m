function [Fx, Fv, Fa] = RobotJac(X, dT, V, a)

L=2.8;

Fx=[1 0 -dT*V*sin(X(3,1));0 1 dT*V*cos(X(3,1));0 0 1];
Fv=[dT*cos(X(3,1));dT*sin(X(3,1));dT*tan(a)/L];
Fa=[0;0;dT*V*(1+(tan(a))^2)/L];

% Fx_rob=[1 0 -dT*V*sin(X(3,1));0 1 dT*V*cos(X(3,1));0 0 1];
% Fx=zeros(9,9);
% Fx(1:3,1:3)=Fx_rob;
% Fx(4:9,4:9)=eye(6);
% 
% Fv_rob=[dT*cos(X(3,1));dT*sin(X(3,1));dT*tan(a)/L];
% Fv=zeros(9,1);
% Fv(1:3,1)=Fv_rob;
% 
% Fa_rob=[0;0;dT*V*(1+(tan(a))^2)/L];
% Fa=zeros(9,1);
% Fa(1:3,1)=Fa_rob;

end

