%% Kalman Filter Lab INS 
%  Developed by Abanob Soliman, UEVE, M2, SAAS.
clc;close all;clear all;
%% Initial conditions
T=0.1;
a0=1;
gma=1; % State Noise (W_k)
omega=0.001;% Observation Noise (V_k)
n=30;
y0=0;y1=-10;
x(:,1)=[y0;y1];%Original initial states
%x(:,1)=[y0;(y1-y0)/T];% Testing Cases initial states
A=[1 T;0 1];
B=[T^2/2;T];
C=[1 0];
G=B;
%% Noise Covariance Matrices Calculation Method 1
%Generate Process Noise (Noise of the state equations)
Var_ONoise = 10e-4;                % Process Noise variance
Mu_ONoise = 0;                     % Process Noise mean
Std_ONoise = sqrt(Var_ONoise)';    % Standard deviation of the Process noise
ONoise = Std_ONoise * randn(2,n) + Mu_ONoise*ones(2,n);    % Gaussian Observation Noise
Q = cov(ONoise');                  % State Noise covariance Matrix
%Generate Observation Noise (Noise of the output equation)
Var_ONoise = 10e-2;                   % Observation Noise variance
Mu_ONoise = 0;                     % Observation Noise mean
Std_ONoise = sqrt(Var_ONoise)';    % Standard deviation of the observation noise
ONoise = Std_ONoise * randn(1,n) + Mu_ONoise*ones(1,n);    % Gaussian Observation Noise
R = cov(ONoise');                  % Observation Noise Covariance Matrix
%% Noise Covariance Matrices Calculation Method 2
Q=gma*G*G';
R=omega*C*C';
%% Measured Position
ym=2;
E=x(:,1)*x(:,1)';
P=E;% The Original initial Variance 
%P=100*[omega 0;0 4*omega/T^2];% The Testing Cases initial Variance
error(1)=ym-C*x(:,1);
%% Iterative Solution
for i=1:n-1
    an=rand(1);
    u=a0+an;
    x(:,i+1)=A*x(:,i)+B*u;
    P=A*P*A'+Q;
    K=P*C'*inv(C*P*C'+R);%% R is the measurement Noise covariance Matrix
    P=(eye(2)-K*C)*P;
    error(i+1)=ym-C*x(:,i+1);
    x(:,i+1)=x(:,i+1)+K*(ym-C*x(:,i+1));
end
%% Plotting the Results
figure
plot(x(1,:));
title('The Estimated Position Values')
xlabel('Number of Iterations')
xlim([1 n])
grid on
figure
plot(x(2,:));
title('The Estimated Velocity Values')
xlabel('Number of Iterations')
xlim([1 n])
grid on
figure
plot(error);
title('The Error in Estimation Curve')
xlabel('Number of Iterations')
xlim([1 n])
grid on