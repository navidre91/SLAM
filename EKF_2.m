
clear
clc

load('data_MAE277_project.mat');
X_main=X;

dT = 0.01; % s % Sampling (100 Hz)
time = 0:dT:20; % Time Vector
N = length(time); % Number of time points
% create variables for storing actual states and measurements
nx = 3; % state dimension
nm = 6; % measurement dimension
nw = 2; % process noise dimension
nv = 6; % measurement noise dimension
Z = zeros(nm,N);
sa = 0.5; % m/s^2 % standard deviation for the process noise (acceleration)
sv = 0.01; % rad^2/s^2 % standard deviation for the measurement Noise
Cw = eye(nw)*sa^2; % process noise covariance (mean is zero)
Cv = eye(nv/3)*sv^2; % measurement noise covariance (mean is zero)
% Initialization of pendulum dynamics
X(:,1) = X_main(:,1); % store actual state evolution in X
X(3,1) = mod(abs(X(3,1)), pi)*sign(X(3,1)); % adjust X(1,1) in [-pi, pi]
% initial condition; normally distributed
x0 = X(:,1); % with mean x0=[0;0] and
Cx= eye(nx)*1e-3; % covariance Cx=[sx1^2 0;0 sx2^2]



% Initialization of Kalman Filter dynamics
% xe denotes the current extended KF state estimate (from update equations)
% and Cex its covariance; xep denotes the current extended state estimate
% from the prediction step and Cxep its covariance;
% xu denotes the current unscented KF state estimate (from update equations)
% and Cux its covariance; xup denotes the current unscented state estimate
% from the prediction step and Cxup its covariance;
Xehat = NaN(nx,N); % store extended KF estimated states in Xehat
msee = NaN(1,N); % store extended KF mse in msee
% intialize filters for first update step
xep=x0; Cxep=Cx;

LM_position=[5;-2;12;0;20;1];
% Simulation of Pendulum dynamics and KF state estimators
% Filters initialize with UPDATE step!
for n = 1:N

%calculate anticipated measurement
X_LM=[xep;LM_position];
Z_i_1=[];

for i = 1:3
  Z_i_1=[Z_i_1;LMmeas(X_LM, i)];
end

%Finding the available measuremnets

z_temp=Zmeas(:,n);
z_availabe=[];
Hv=[];
z=[];
z_i_1=[];

for i = 1:length(z_temp)/2

    if ~isnan(z_temp(2*i,1))
        z_availabe=[z_availabe;i];
        Hv=[Hv;eye(2)];
        z=[z;z_temp(2*i-1:2*i,1)];
        z_i_1=[z_i_1;Z_i_1(2*i-1:2*i,1)];
    end

end

% EKF---------------------------------------------------------------

%calculating jacobian of measurment wrt to states

if ~isempty(z_availabe)

Hi = [];
for i = 1:length(z_availabe)
  Hi=[Hi;LMJac(X_LM, z_availabe(i))];
end

Czep=Hi*Cxep*Hi'+Hv*Cv*Hv'+0.001*eye(size(Hi,1));
Cxzep=Cxep*Hi';

K = Cxzep*Czep^-1; % Kalman gain matrix
xe = xep + K*(z - z_i_1);

xe(3,1) = mod(abs(xe(3,1)), pi)*sign(xe(3,1)); %check to be in [-pi pi] range

Cxe = Cxep - K*Czep*K';
% Store current state estimate and mse
Xehat(:,n) = xe;
msee(n) = trace(Cxe);

end

if isempty(z_availabe)
    xe = xep;
    xe(3,1) = mod(abs(xe(3,1)), pi)*sign(xe(3,1));
    Xehat(:,n) = xe;
end

% Prediction Step in Extended KF


xep = RobotDyn(xe,dT,Vhat(n),ahat(n)); % nonlinear robot dynamics

% Jacobian of State dynamics wrt w and x
[Fx, Fv, Fa] = RobotJac(xe, dT, Vhat(n), ahat(n));

G=[Fv Fa];

Cxep = Fx*Cxe*Fx' + G*Cw*G';
end

MSE_EKF=mean(msee); % average EKF MS tracking error
disp(['EKF: Trial average Mean Square tracking error=',num2str(MSE_EKF)])

simulation_noisy(X,Vhat,ahat)

plot(Xehat(1,:),Xehat(2,:))