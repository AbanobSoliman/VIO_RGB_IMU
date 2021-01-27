function [States,qw_i] = IMU_Propagation_(States,T,accel,gyro,qw_i,ref)

    % First Order Quaternion Integrator
    Omega_curr = [0,-gyro.curr(1),-gyro.curr(2),-gyro.curr(3);gyro.curr(1),0,gyro.curr(3),-gyro.curr(2);gyro.curr(2),-gyro.curr(3),0,gyro.curr(1);gyro.curr(3),gyro.curr(2),-gyro.curr(1),0];
    Omega_prev = [0,-gyro.prev(1),-gyro.prev(2),-gyro.prev(3);gyro.prev(1),0,gyro.prev(3),-gyro.prev(2);gyro.prev(2),-gyro.prev(3),0,gyro.prev(1);gyro.prev(3),gyro.prev(2),-gyro.prev(1),0];
    Omega_avrg = [0,-gyro.avrg(1),-gyro.avrg(2),-gyro.avrg(3);gyro.avrg(1),0,gyro.avrg(3),-gyro.avrg(2);gyro.avrg(2),-gyro.avrg(3),0,gyro.avrg(1);gyro.avrg(3),gyro.avrg(2),-gyro.avrg(1),0];
    term1 = 0.5*Omega_avrg*T;
    term2 = eye(4)+term1+0.5*term1^2+(1/6)*term1^3+(1/24)*term1^4;
    qw_i  = (term2+(T^2/48)*(Omega_curr*Omega_prev-Omega_prev*Omega_curr))*qw_i';
    qw_i  = qw_i';
    
    States(1:3)   = States(1:3)+States(4:6)*T;
    
    rf = rfconfig(ref);
    g = zeros(1,3);
    g(rf.GravityIndex) = -rf.GravitySign*rf.GravityAxisSign*gravms2();
    newAccel      = rotatepoint(quaternion(qw_i), accel) - g;
    States(4:6)   = States(4:6)+newAccel.'*T;   
    
    States(7:10)  = qw_i;          %estimated^qw_i
    States(11:13) = States(11:13);
    States(14:16) = States(14:16);
    States(17:20) = States(17:20);
    States(21:24) = States(21:24); %estimated^qi_c
    States(25:27) = States(25:27);
    States(28:31) = States(28:31); %estimated^qv_w
end

function rf = rfconfig(refStr)
%RFCONFIG Return the reference frame configuration object based on the 
%   reference frame string.
rf = fusion.internal.frames.ReferenceFrame.getMathObject( ...
                refStr);
end

function g = gravms2()
    g = fusion.internal.UnitConversions.geeToMetersPerSecondSquared(1);
end
