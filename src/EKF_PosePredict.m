%% IMU and CAM Fusion for Inertial Navigation using Error States Extended Kalman Filter
% This example shows how we might build an IMU + Monocular Camera fusion algorithm
% suitable for Unmanned Aerial Aehicles (UAVs) or Quadcopters. 
% This example uses accelerometers, gyroscopes and CAM to determine orientation and position of a UAV.
% Developed by Abanob Soliman for Master's Thesis Under Supervision
% of Prof. Dr. Samia Bouchafa Bruneau Head of IBISC Laboratory 
% Copyrights reserved 2019-2020 for University Of Evry, Laboratoire IBISC, Pelvoux, France

function [fusionfilt] = EKF_PosePredict (fusionfilt, accel, gyro, orient, transl, t)
    
    T      = fusionfilt.T;
    States = fusionfilt.State;
    R      = fusionfilt.R;
    Qc     = fusionfilt.Qc;
    P      = fusionfilt.StateCovariance;
    Cqw_i  = fusionfilt.Cqw_i;
    Cqi_c  = fusionfilt.Cqi_c;
    qw_i   = fusionfilt.qw_i;
    qi_c   = fusionfilt.qi_c;
    
    % Current Camera Readings (Sfm Vision Algorithm Output)
    mpv_c = transl;                   % pv_c
    mqv_c = rotm2quat (orient);       % qv_c
    
    qv_w  = quatmultiply(rotm2quat(orient),quatmultiply(qi_c,quatinv(qw_i)));
    Cqv_w = quat2rotm(qv_w);  % Rotation Matrix of Vision Coord. wrt. World Coord.
    
    % Current Accelerometer Readings
    am_curr = accel.curr';   % Current Accel. Readings
    am_skew_curr = [0,am_curr(3),-am_curr(2);-am_curr(3),0,am_curr(1);am_curr(2),-am_curr(1),0];
        
    wm_curr = gyro.curr';                    % Current Gyro. Readings
    wmskew_curr = [0,wm_curr(3),-wm_curr(2);-wm_curr(3),0,wm_curr(1);wm_curr(2),-wm_curr(1),0]; 
    
    % Using the current data (t)
    a_skew_estimated = am_skew_curr - [0,States(13),-States(12);-States(13),0,States(11);States(12),-States(11),0];
    wskew_estimated  = wmskew_curr  - [0,States(16),-States(15);-States(16),0,States(14);States(15),-States(14),0];
    
    % Step : 0 -> States Propagation for the current time step
    States(1:3)   = States(1:3)+T*States(4:6)+0.5*T^2*(Cqw_i*(accel.curr'-States(11:13))-[0;0;9.80665]);
    States(4:6)   = States(4:6)+T*(Cqw_i*(accel.curr'-States(11:13))-[0;0;9.80665]);       
    States(7:10)  = qw_i;          %estimated^qw_i
    States(11:13) = States(11:13);
    States(14:16) = States(14:16);
    States(17:20) = States(17:20);
    States(21:24) = States(21:24); %estimated^qi_c
    States(25:27) = States(25:27);
    States(28:31) = States(28:31); %estimated^qv_w
    
    % Step : 1 -> States Estimation [[x~]]
    A = -Cqw_i*a_skew_estimated*((t^2/2)-(t^3/factorial(3))*wskew_estimated+(t^4/factorial(4))*wskew_estimated^2);
    B = -Cqw_i*a_skew_estimated*(-(t^3/factorial(3))+(t^4/factorial(4))*wskew_estimated-(t^5/factorial(5))*wskew_estimated^2);
    C = -Cqw_i*a_skew_estimated*(t-(t^2/factorial(2))*wskew_estimated+(t^3/factorial(3))*wskew_estimated^2);
    D = -A;
    E = eye(3)-t*wskew_estimated+(t^2/factorial(2))*wskew_estimated^2;
    F = -t+(t^2/factorial(2))*wskew_estimated-(t^3/factorial(3))*wskew_estimated^2;

    Fd_symb = [eye(3),t*eye(3),A,B,-Cqw_i*(t^2/2),zeros(3,13);zeros(3),eye(3),C,D,-Cqw_i*t,zeros(3,13);zeros(3),zeros(3),E,F,zeros(3),zeros(3,13);zeros(3),zeros(3),zeros(3),eye(3),zeros(3),zeros(3,13);zeros(3),zeros(3),zeros(3),zeros(3),eye(3),zeros(3,13);zeros(13,3),zeros(13,3),zeros(13,3),zeros(13,3),zeros(13,3),eye(13)];
    Fd = double(subs(Fd_symb,t,T));
    Gd = [zeros(3),zeros(3),zeros(3),zeros(3);-Cqw_i,zeros(3),zeros(3),zeros(3);zeros(3),zeros(3),-eye(3),zeros(3);zeros(3),zeros(3),zeros(3),eye(3);zeros(3),eye(3),zeros(3),zeros(3);zeros(13,3),zeros(13,3),zeros(13,3),zeros(13,3)];  
    
    % Step : 2 -> Error States Covariance Estimation [[P]]
    const = Gd*Qc*Gd';
    func = Fd_symb*const*Fd_symb';
    Q11_sym = func(1:14,1:14);
    Q11_int = int(Q11_sym,0,T);
    Q12_sym = func(1:14,15:28);
    Q12_int = int(Q12_sym,0,T);
    Q22_sym = func(15:28,15:28);
    Q22_int = int(Q22_sym,0,T);
    Qd = double([Q11_int,Q12_int;Q12_int',Q22_int]);

    P = Fd*P*Fd'+Qd;

    % Step : 3-> Measurement Model [[H]] and the Residual [[Z_tilda]]
    Zp_tilda = mpv_c' - Cqv_w*(States(1:3)+Cqw_i*States(18:20))*States(17);
    Zq_tilda = quatmultiply(mqv_c,quatinv(quatmultiply(qi_c,quatmultiply(qw_i,qv_w))));
    Ze_tilda = quat2eul(Zq_tilda);
    Z_tilda  = [Zp_tilda;Ze_tilda'];

    v1 = States(18:20);
    v1_skew = [0,v1(3),-v1(2);-v1(3),0,v1(1);v1(2),-v1(1),0];
    v2 = (States(1:3)+Cqw_i*States(18:20))*States(17);
    v2_skew = [0,v2(3),-v2(2);-v2(3),0,v2(1);v2(2),-v2(1),0];
    
    Hp = [Cqv_w*States(17),zeros(3),-Cqv_w*Cqw_i*v1_skew*States(17),zeros(3),...
            zeros(3),Cqv_w*Cqw_i*v1+Cqv_w*States(1:3),...
                Cqv_w*Cqw_i*States(17),zeros(3),-Cqv_w*v2_skew,zeros(3)];

    h_qw_i = Cqw_i;
    h_qi_c = Cqi_c;
    h_qv_w = Cqv_w;
    Hq = [zeros(3,6),h_qw_i,zeros(3,10),h_qi_c,zeros(3,3),h_qv_w];

    H = [Hp;Hq];
    
    % Step : 4 -> Compute the Innovation Term [[S]]
    S = H*P*H' + R;
    
    % Step : 5 -> Kalman Filter Gain [[K]]
    K = P*H'/S;

    % Step : 6 -> Error States Update Estimation [[x~^]]
    ErrorStates = K*(Z_tilda);

    % Step : 7 -> Covariance Update Estimation [[P^]]
    P = (eye(28)-K*H)*P*(eye(28)-K*H)'+K*R*K';
    
    % Step : 8 -> States Update for the next time step
%     States(1:3)   = mpv_c';
%     States(4:6)   = States(4:6);
    States(1:3)   = States(1:3)   - ErrorStates(1:3);
    States(4:6)   = States(4:6)   - ErrorStates(4:6);
    States(11:20) = States(11:20) - ErrorStates(10:19);
    States(18:20) = States(18:20) - ErrorStates(17:19);
    States(25:27) = States(25:27) - ErrorStates(23:25);

    d_qw_i = double([1,.5*ErrorStates(7),.5*ErrorStates(8),.5*ErrorStates(9)]);
    d_qi_c = double([1,.5*ErrorStates(20),.5*ErrorStates(21),.5*ErrorStates(22)]);
    d_qv_w = double([1,.5*ErrorStates(26),.5*ErrorStates(27),.5*ErrorStates(28)]);

    States(7:10)  = quatmultiply(d_qw_i,States(7:10)')  ; % Estimated^qw_i
    States(21:24) = quatmultiply(d_qi_c,States(21:24)') ; % Estimated^qi_c
    States(28:31) = quatmultiply(d_qv_w,States(28:31)') ; % Estimated^qv_w

    % Update the fusion filter for the next 3D-POSE estimation
    fusionfilt.State           = States;
    fusionfilt.ErrorState      = ErrorStates;
    fusionfilt.StateCovariance = P;
    fusionfilt.qi_c            = States(21:24)'; 
    fusionfilt.Cqi_c           = quat2rotm(fusionfilt.qi_c);

    disp("Fusion for this Camera frame done Successfully!");

end