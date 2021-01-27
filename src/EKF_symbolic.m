clc;
sym('Cqw_i', [3,3]);
sym('a_skew_estimated', [3,3]);
sym('wskew_estimated', [3,3]);
syms T   
accelerometer_random_walk   =  (3.0000e-3)^2*ones(1,3);    % sigma_nba->[m/s^3/sqrt(Hz)](accel bias diffusion)
accelerometer_noise_density =  (2.0000e-3)^2*ones(1,3);    % sigma_na->[m/s^2/sqrt(Hz)](accel "white noise")
gyroscope_random_walk       =  (1.9393e-05)^2*ones(1,3);   % sigma_nbw->[rad/s^2/sqrt(Hz)](gyro bias diffusion)
gyroscope_noise_density     =  (1.6968e-04)^2*ones(1,3);   % sigma_nw->[rad/s/sqrt(Hz)](gyro "white noise")

dQc = [accelerometer_noise_density,accelerometer_random_walk, ...
    gyroscope_noise_density,gyroscope_random_walk];

Qc = diag(dQc);
Gc = [zeros(3),zeros(3),zeros(3),zeros(3);-Cqw_i',zeros(3),zeros(3),zeros(3);zeros(3),zeros(3),-eye(3),zeros(3);zeros(3),zeros(3),zeros(3),eye(3);zeros(3),eye(3),zeros(3),zeros(3);zeros(13,3),zeros(13,3),zeros(13,3),zeros(13,3)];  
        
A = -Cqw_i'*a_skew_estimated*((T^2/2)-(T^3/factorial(3))*wskew_estimated+(T^4/factorial(4))*wskew_estimated^2);
B = -Cqw_i'*a_skew_estimated*(-(T^3/factorial(3))+(T^4/factorial(4))*wskew_estimated-(T^5/factorial(5))*wskew_estimated^2);
C = -Cqw_i'*a_skew_estimated*(T-(T^2/factorial(2))*wskew_estimated+(T^3/factorial(3))*wskew_estimated^2);
D = -A;
E = eye(3)-T*wskew_estimated+(T^2/factorial(2))*wskew_estimated^2;
F = -T+(T^2/factorial(2))*wskew_estimated-(T^3/factorial(3))*wskew_estimated^2;

Fd_symb = [eye(3),T*eye(3),A,B,-Cqw_i'*(T^2/2),zeros(3,13);zeros(3),eye(3),C,D,-Cqw_i'*T,zeros(3,13);zeros(3),zeros(3),E,F,zeros(3),zeros(3,13);zeros(3),zeros(3),zeros(3),eye(3),zeros(3),zeros(3,13);zeros(3),zeros(3),zeros(3),zeros(3),eye(3),zeros(3,13);zeros(13,3),zeros(13,3),zeros(13,3),zeros(13,3),zeros(13,3),eye(13)]
const = Gc*Qc*Gc';
Qd_symb1 = int(Fd_symb,T)
Qd_symb = Qd_symb1*const*Qd_symb1'