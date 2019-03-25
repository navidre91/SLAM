function [X_rec]=simulation_noisy(X,Vhat,ahat)


dT=0.01;

X0=zeros(9,1);
X0(1:3,1)=X(:,1);
X_rec=[];

for i=1:length(Vhat)
    
    X1 = RobotDyn(X0, dT, Vhat(i), ahat(i));
    X_rec(:,i)=X1;
    X0=X1;
    
end





end

