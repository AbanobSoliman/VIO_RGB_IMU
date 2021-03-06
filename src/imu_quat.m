function [q_imu]=imu_quat(ax_imu,ay_imu,az_imu,gx_imu,gy_imu,gz_imu,t_imu)
q_imu = zeros(4,length(t_imu));
ax1 = mean(ax_imu(1:100));
ay1 = mean(ay_imu(1:100));
az1 = mean(az_imu(1:100));
pitch1 = atan2(-ax1,sqrt(ay1^2 + az1^2));
roll1 = atan2(ay1,az1);
pitch1 = (180/pi)*(pitch1 .* (pitch1>=0) + (pitch1 + 2*pi) .*(pitch1<0));
roll1 = (180/pi)*(roll1 .* (roll1>=0) + (roll1 + 2*pi) .*(roll1<0));
yaw1 = 1;
cy = cosd(0.5*yaw1);
sy = sind(0.5*yaw1);
cr = cosd(0.5*roll1);
sr = sind(0.5*roll1);
cp = cosd(0.5*pitch1);
sp = sind(0.5*pitch1);
q_0 = cy * cr * cp + sy * sr * sp;
q_1 = cy * sr * cp - sy * cr * sp;
q_2 = cy * cr * sp + sy * sr * cp;
q_3 = sy * cr * cp - cy * sr * sp;
q_imu(:,1) = [q_0; q_1; q_2; q_3];
dt_imu = t_imu(2:end)-t_imu(1:end-1);
for i = 1:length(dt_imu)
    b = [1/6 1/3  1/3 1/6];
    d = [1 1/2 1/2 1];
    k = zeros(4,4);
    k(:,1) = (dt_imu(i)) * runge_orient( [gx_imu(i), gy_imu(i), gz_imu(i)], q_imu(:,i)); %-gx_imu(i), gz_imu(i), gy_imu(i)
    for j = 2:4
        
        k(:,j) = (dt_imu(i)) * runge_orient([gx_imu(i), gy_imu(i), gz_imu(i)], q_imu(:,i) + (d(j)*k(:,j-1)));

    end
    q_imu(:,i+1) = q_imu(:,i) + (b*k')';
    q_norm = sqrt(q_imu(1,i+1)^2 + q_imu(2,i+1)^2 + q_imu(3,i+1)^2 + q_imu(4,i+1)^2);
    q_imu(:,i+1) = q_imu(:,i+1)/q_norm;
end
end
