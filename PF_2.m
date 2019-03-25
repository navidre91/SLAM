
clearvars -except Xehat
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
sv = 0.5; % rad^2/s^2 % standard deviation for the measurement Noise
Cw = eye(nw)*sa^2; % process noise covariance (mean is zero)
Cv = eye(nv)*sv^2; % measurement noise covariance (mean is zero)
% Initialization of pendulum dynamics
X(:,1) = X_main(:,1); % store actual state evolution in X
X(3,1) = mod(abs(X(3,1)), pi)*sign(X(3,1)); % adjust X(1,1) in [-pi, pi]
% initial condition; normally distributed
x0 = X(:,1); % with mean x0=[0;0] and
Cx= eye(nx)*1e-3; % covariance Cx=[sx1^2 0;0 sx2^2]

Hv=eye(6);

% Initialization of Kalman Filter dynamics
% xe denotes the current extended KF state estimate (from update equations)
% and Cex its covariance; xep denotes the current extended state estimate
% from the prediction step and Cxep its covariance;
% xu denotes the current unscented KF state estimate (from update equations)
% and Cux its covariance; xup denotes the current unscented state estimate
% from the prediction step and Cxup its covariance;
Xphat = NaN(nx,N); % store extended KF estimated states in Xehat
msee = NaN(1,N); % store extended KF mse in msee
% intialize filters for first update step
xep=x0; Cxep=Cx;

LM_position=[5;-2;12;0;20;1];
% Simulation of Pendulum dynamics and KF state estimators
% Filters initialize with UPDATE step!

M = 2000; % number of particles
Mlow = 0; % threshold of effective particles to perform resampling
% set Mlow = 0 to do resampling at each step

% Initialize PF particles
Xpt = mvnrnd(x0',Cx,M)'; % particles are stored in columns of Xpt
Wpt = ones(1,M)*(1/M); % initialize importance weights
Xtemp = zeros(nx,M); % temp storage for resamped particles
Mtemp = zeros(1,M); % temp storage for resampling indices



for n = 1:N


z_temp=Zmeas(:,n);
z_availabe=[];
Cv_temp=[];
z=[];

for i = 1:length(z_temp)/2

    if ~isnan(z_temp(2*i,1))
        z_availabe=[z_availabe;i];
        z=[z;z_temp(2*i-1:2*i,1)];
    end

end    

Cv_temp=Cv(1:2*length(z_availabe),1:2*length(z_availabe)); 

if ~isempty(z_availabe)

% PF---------------------------------------------------------------
for k=1:M
 
%calculate anticipated measurement
X_LM=[Xpt(:,k);LM_position];
Z_i_1=[];

for i = 1:length(z_availabe)
  Z_i_1=[Z_i_1;LMmeas(X_LM, z_availabe(i))];
end

%calculating measurment, where measurement is NaN we set to be equal to
%anticipated measurement




Wpt(1,k) = Wpt(1,k)* mvnpdf(z,Z_i_1,Cv_temp);

end

Wpt = Wpt/sum(Wpt);
% Resample if necessary
Meff = 1/norm(Wpt); % effective number of particles
if Meff < Mlow
    Wtemp = cumsum(Wpt); % cumulative sum of weights
    for m=1:M
        m1 = find(rand() < Wtemp,1); % select sample
        if m1>M, keyboard,end
        Mtemp(m) = m1; % store index of selected particle
        Xtemp(:,m) = Xpt(:,m1);
    end
%     Muq = length(unique(Mtemp)); % # unique particles after resampling
    Xpt = Xtemp; % resampled particles
 % reset weights
 Wpt = ones(1,M)*(1/M);

end

end

Wpt = ones(1,M)*(1/M);
Xphat(:,n) = Xpt*Wpt'; % average particles after update step and
% resampling to create PF point estimate
% Prediction step: pass particles through 

w1=mvnrnd(0,sa^2,M);
w2=mvnrnd(0,sa^2,M);

for j=1:M    
Xpt(:,j) = RobotDyn(Xpt(:,j),dT,Vhat(n)+w1(j),ahat(n)+w2(j)); % particles processed through dynamics
end

end



X_noisy=simulation_noisy(X,Vhat,ahat);
% 
plot(Xphat(1,:),Xphat(2,:))