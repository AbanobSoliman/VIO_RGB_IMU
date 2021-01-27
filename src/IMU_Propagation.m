function [States,qw_i] = IMU_Propagation(States,T,accel,gyro,qw_i,g)

    q0 = States(7);
    q1 = States(8);
    q2 = States(9);
    q3 = States(10);
    
    bax = States(14);
    bay = States(15);
    baz = States(16);
  
    ax = accel(1);
    ay = accel(2);
    az = accel(3);
    
    gN = 0;
    gE = 0;
    gD = g;

    States(11:13) = States(11:13); %bias_gyro
    States(14:16) = States(14:16); %bias_accel
    States(17:20) = States(17:20); %scale_pi_c
    States(21:24) = States(21:24); %qi_c
    States(25:27) = States(25:27); %pv_w
    States(28:31) = States(28:31); %qv_w   
    
    States(1:3)   = States(1:3)+T*States(4:6); %pw_i
    
    States(4)     = States(4)-T*(q0*(q0*(bax-ax)-q3*(bay-ay)+q2*(baz-az))+gN+...
                               q2*(q1*(bay-ay)-q2*(bax-ax)+q0*(baz-az))+...
                               q1*(q1*(bax-ax)+q2*(bay-ay)+q3*(baz-az))-...
                               q3*(q3*(bax-ax)+q0*(bay-ay)-q1*(baz-az)));       %vw_i_x
    States(5)     = States(5)-T*(q0*(q3*(bax-ax)+q0*(bay-ay)-q1*(baz-az))+gE-...
                               q1*(q1*(bay-ay)-q2*(bax-ax)+q0*(baz-az))+...
                               q2*(q1*(bax-ax)+q2*(bay-ay)+q3*(baz-az))+...
                               q3*(q0*(bax-ax)-q3*(bay-ay)+q2*(baz-az)));       %vw_i_y
    States(6)     = States(6)-T*(q0*(q1*(bay-ay)-q2*(bax-ax)+q0*(baz-az))+gD+...
                               q1*(q3*(bax-ax)+q0*(bay-ay)-q1*(baz-az))-...
                               q2*(q0*(bax-ax)-q3*(bay-ay)+q2*(baz-az))-...
                               q3*(q1*(bax-ax)+q2*(bay-ay)+q3*(baz-az)));       %vw_i_z
                           
     % First Order Quaternion Integrator
    Omega_curr = [0,-gyro.curr(1),-gyro.curr(2),-gyro.curr(3);gyro.curr(1),0,gyro.curr(3),-gyro.curr(2);gyro.curr(2),-gyro.curr(3),0,gyro.curr(1);gyro.curr(3),gyro.curr(2),-gyro.curr(1),0];
    Omega_prev = [0,-gyro.prev(1),-gyro.prev(2),-gyro.prev(3);gyro.prev(1),0,gyro.prev(3),-gyro.prev(2);gyro.prev(2),-gyro.prev(3),0,gyro.prev(1);gyro.prev(3),gyro.prev(2),-gyro.prev(1),0];
    Omega_avrg = [0,-gyro.avrg(1),-gyro.avrg(2),-gyro.avrg(3);gyro.avrg(1),0,gyro.avrg(3),-gyro.avrg(2);gyro.avrg(2),-gyro.avrg(3),0,gyro.avrg(1);gyro.avrg(3),gyro.avrg(2),-gyro.avrg(1),0];
    term1 = 0.5*Omega_avrg*T;
    term2 = eye(4)+term1+0.5*term1^2+(1/6)*term1^3+(1/24)*term1^4;
    qw_i  = (term2+(T^2/48)*(Omega_curr*Omega_prev-Omega_prev*Omega_curr))*qw_i';
    qw_i  = qw_i';
    States(7:10)  = qw_i;          %estimated^qw_i
end