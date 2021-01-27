%% Reading OKVIS Data 
Tbl_OKVIS = readtable('OKVIS_state_estimation.csv');
Tbl_cam = readtable('data_cam.csv');
camTimeSteps = Tbl_cam.x_timestamp_ns_;
camTimeSteps  = (camTimeSteps - camTimeSteps(1))*1e-9;
OKVISPositions    = [Tbl_OKVIS.p_x,Tbl_OKVIS.p_y,Tbl_OKVIS.p_z];
OKVISQuaternions  = quaternion([Tbl_OKVIS.q_w,Tbl_OKVIS.q_x,Tbl_OKVIS.q_y,Tbl_OKVIS.q_z]);
OKVISOrientations = quat2eul(OKVISQuaternions,'ZYX')*180/pi; % In Degrees
OKVISSteps = size(Tbl_OKVIS,1);
%% Reading the GroundTruth Data 
Tbl_gt = readtable('data_gt.csv');
gtTimeSteps = Tbl_gt.x_timestamp_ns_;
gtTimeSteps  = (gtTimeSteps - gtTimeSteps(1))*1e-9;
gtPositions    = [Tbl_gt.p_RS_R_x_m_,Tbl_gt.p_RS_R_y_m_,Tbl_gt.p_RS_R_z_m_];
gtQuaternions  = quaternion([Tbl_gt.q_RS_w__,Tbl_gt.q_RS_x__,Tbl_gt.q_RS_y__,Tbl_gt.q_RS_z__]);
gtOrientations = quat2eul(gtQuaternions,'ZYX')*180/pi; % In Degrees
gtSteps = size(Tbl_gt,1);
%% Calculating the root mean square errors
k_gt = 1;
for f=1:OKVISSteps
    disttt  = abs(camTimeSteps(k_gt)-gtTimeSteps);
    k_imu = find(disttt == min(disttt));
    if f == k_imu
        posd(k_gt,:) = OKVISPositions(f,:) - gtPositions(k_gt,:);
        quatd(k_gt)  = rad2deg(dist(OKVISQuaternions(f,1), gtQuaternions(k_gt,1)) );
        k_gt = k_gt + 1;
        if k_gt > OKVISSteps
            break
        end
    end
end
% Display RMS errors in the command window.
fprintf('\n\nEnd-to-End Simulation Position RMS Error\n');
msep = sqrt(mean(posd.^2));
fprintf('\tX: %.2f , Y: %.2f, Z: %.2f   (meters)\n\n',msep(1), ...
    msep(2), msep(3));

fprintf('End-to-End Quaternion Distance RMS Error (degrees) \n');
fprintf('\t%.2f (degrees)\n\n', sqrt(mean(quatd.^2)));