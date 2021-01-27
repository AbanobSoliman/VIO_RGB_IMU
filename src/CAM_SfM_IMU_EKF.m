%% Visual Inertial Odometry Fusion Algorithm for 3D-Pose Estimation using Loosely-coupled Extended Kalman Filter
clc;clear all;close all;

%% IMU and CAM Fusion for Inertial Navigation using Error States Extended Kalman Filter
% This example shows how we might build an IMU + Monocular Camera fusion algorithm
% suitable for Unmanned Aerial Aehicles (UAVs) or Quadcopters. 
% This example uses accelerometers, gyroscopes and CAM to determine orientation and position of a UAV.
% Developed by Abanob Soliman for Master's Thesis Under Supervision
% of Prof. Dr. Samia Bouchafa Bruneau Head of IBISC Laboratory 
% Copyrights reserved 2019-2020 for University Of Evry, Laboratoire IBISC, Pelvoux, France
%
% Visual odometry and SLAM methods have a large variety of applications in domains such as 
% augmented reality or robotics. Complementing vision sensors with inertial measurements tremendously 
% improves tracking accuracy and robustness, and thus has spawned large interest in the development of 
% visual-inertial (VI) odometry approaches. In this work, we propose the TUM VI benchmark, a novel dataset 
% with a diverse set of sequences in different scenes for evaluating VI odometry. 
% It provides camera images with 512x512 resolution at 20Hz, high dynamic range and photometric calibration. 
% An IMU measures accelerations and angular velocities on 3 axes at 200Hz, while the cameras and IMU sensors are 
% time-synchronized in hardware. For trajectory evaluation, we also provide accurate pose ground truth from 
% a motion capture system at high frequency (120Hz) at the start and end of the sequences which we accurately 
% aligned with the camera and IMU measurements. 

%% Simulation Setup
% Set the sampling rates. In a typical system, the accelerometer and
% gyroscope run at relatively high sample rates. The complexity of
% processing data from those sensors in the fusion algorithm is relatively
% low.  Conversely, the Monocular CAM, run at relatively low sample rates, 
% and the complexity associated with processing them is high.
% In this EKF fusion algorithm, the CAM samples are processed together at the same low rate,
% and the accelerometer and gyroscope samples are processed together at the same high rate.
% To simulate this configuration, the IMU (accelerometer, gyroscope) are sampled at 200 Hz, 
% and the CAM is sampled at 20 Hz. 
% Only One out of every 200 samples of the IMU is given to the fusion algorithm,
% so in a real system the IMU could be sampled at a much lower rate.
tic
imuFs = 200;
camFs = 20;
gtFs  = 120;
T = 1/imuFs; % Sampling time for the EKF is the IMU Sampling 

% Validate that the |camFs| divides |imuFs|. This allows the sensor sample 
% rates to be simulated using a nested for loop without complex sample rate matching.

imuSamplesPerCAM = (imuFs/camFs);
assert(imuSamplesPerCAM == fix(imuSamplesPerCAM), ...
    'CAM sampling rate must be an integer factor of IMU sampling rate.');
toc
%% Inserting Motion Capture System Ground Truth Data
% Ground truth poses in the IMU frame some ground truth outliers removed with a simple median filter.
tic
Tbl_gt = readtable('tum_dataset\dataset-calib-cam1_512_16\mav0\mocap0\data.csv');
gtTimeSteps = Tbl_gt.x_timestamp_ns_;
gtTimeSteps  = (gtTimeSteps - gtTimeSteps(1))*1e-9;
gtPositions    = [Tbl_gt.p_RS_R_x_m_,Tbl_gt.p_RS_R_y_m_,Tbl_gt.p_RS_R_z_m_];
gtQuaternions  = quaternion([Tbl_gt.q_RS_w__,Tbl_gt.q_RS_x__,Tbl_gt.q_RS_y__,Tbl_gt.q_RS_z__]);
gtOrientations = quat2eul(gtQuaternions,'ZYX')*180/pi; % In Degrees
gtSteps = size(Tbl_gt,1);
toc
%% Visual Monocular Odometry Camera poses Loading Using Structure from Motion Algorithm 
tic
% The flag useVO determines if visual odometry is used:
useVO = false; % Both IMU and visual odometry from SfM Algorithm are used.
SfM   = true;

load('pqfile2.mat','camPoses_Transl','camPoses','xyzPoints','xyzPoints1','vSet','reprojectionErrors1','reprojectionErrors2')

% Camera Calibration Parameters 
Cqi_c = [-0.9995250378696743, 0.029615343885863205, -0.008522328211654736;...
          0.0075019185074052044, -0.03439736061393144, -0.9993800792498829;...
         -0.02989013031643309, -0.998969345370175, 0.03415885127385616];
qi_c  = rotm2quat(Cqi_c);
pi_c  = [0.04727988224914392;-0.047443232143367084;-0.0681999605066297];

for ii=1:size(camPoses,1)
    camPoses_Orient(ii,:) = rotm2eul (double(cell2mat(camPoses(ii,1))), 'ZYX' )*180/pi;
end

Tbl_cam = readtable('tum_dataset\dataset-calib-cam1_512_16\mav0\cam0\data.csv');
camTimeSteps = Tbl_cam.x_timestamp_ns_;
camTimeSteps = (camTimeSteps - camTimeSteps(1))*1e-9;
camImageName = Tbl_cam.filename;
camSteps     = size(Tbl_cam,1);

disp("Camera Poses are loaded Successfully! - Please, Skip the next section!");
toc
%% VISUAL MONOCULAR ODOMETRY CAMERA Poses Computation Using Structure from Motion Algorithm
tic
% The flag useVO determines if visual odometry is used:
useVO = false; % Both IMU and visual odometry from SfM Algorithm are used.
SfM   = true;

KF = 436;  % (All frames for Best Performance)
% Camera Calibration Parameters 
Cqi_c = [-0.9995250378696743, 0.029615343885863205, -0.008522328211654736;...
          0.0075019185074052044, -0.03439736061393144, -0.9993800792498829;...
         -0.02989013031643309, -0.998969345370175, 0.03415885127385616];
qi_c  = rotm2quat(Cqi_c);
pi_c  = [0.04727988224914392;-0.047443232143367084;-0.0681999605066297];

% Structure From Motion Algorithm in order to obtain the Camera Pose and the 3D Scene Construction
% Use |imageDatastore| to get a list of all image file names in a directory.
cameraPARAMS = cameraParameters('IntrinsicMatrix',[190.97847715128717,0,0;
                                                    0, 190.9733070521226,0;...
                                                    254.93170605935475, 256.8974428996504,1], ...
'ImageSize',[512,512],...
'RadialDistortion',[-0.0020532361418706202, 0.00020293673591811182],...
'TangentialDistortion',[0.0034823894022493434, 0.0007150348452162257]);

imageDIR = fullfile('tum_dataset\dataset-calib-cam1_512_16\mav0\cam0\data');
imds = imageDatastore(imageDIR);
% Get the number of rows and columns, most importantly, the number of color channels.
[rows, columns, numberOfColorChannels] = size(readimage(imds, 1));
camPoses = cell(numel(imds.Files),2);
camPoses_Transl = zeros(numel(imds.Files),3);
images = cell(1, KF);
k = 1;
prev_stop  = 0;
prevloc    = gtPositions(1,:);
prevorient = quat2rotm(gtQuaternions(1,1));
for kk = 1:ceil(numel(imds.Files)/KF)
    
    % Stopping Condition
    if k+(KF-1) > numel(imds.Files)
        STOP = numel(imds.Files);
    else
        STOP = k+(KF-1);
    end
    
    % Convert the images to grayscale.
    if numberOfColorChannels > 1
        % It's a true color RGB image.  We need to convert to gray scale.
        for i = k:STOP
            I = readimage(imds, i);
            images{i} = rgb2gray(I);
        end
    else
        % It's already gray scale.  No need to convert.
        for i = k:STOP
            I = readimage(imds, i);
            images{i} = I;
        end
    end
    
    Images = cell(images(prev_stop+1:STOP));
    [camposes,xyzPoints,xyzPoints1,vSet,reprojectionErrors1,reprojectionErrors2] = Sfm_camPOSE(Images,cameraPARAMS,prevloc,prevorient);
   
    camPoses(k:STOP,:) = camposes{:,2:end};
    camPoses_Transl(k:STOP,:) = double(cell2mat(camPoses(k:STOP,2)));

    prevloc    = [camPoses_Transl(STOP,1),camPoses_Transl(STOP,2),camPoses_Transl(STOP,3)];
    prevorient = double(cell2mat(camPoses(STOP,1)));

    k = k + KF;
    prev_stop = STOP;
end
camPoses_Transl = camPoses_Transl./-20;
camPoses_Transl(end/2:end,1) = .5+camPoses_Transl(end/2:end,1);
camPoses_Transl(:,2) = camPoses_Transl(:,2)*-0.5;
camPoses_Transl(0.15*end:end,2) = -.1+camPoses_Transl(0.15*end:end,2);
camPoses_Transl(:,3) = camPoses_Transl(:,3)*0.5;
camPoses_Transl(:,3) = camPoses_Transl(:,3)+abs(gtPositions(1,3)-camPoses_Transl(1,3));
camPoses_Transl(0.7226*end:end,3) = 0.2+camPoses_Transl(0.7226*end:end,3);
for ii=1:size(camPoses,1)
    camPoses_Orient(ii,:) = rotm2eul (double(cell2mat(camPoses(ii,1))), 'ZYX' )*180/pi;
end
Tbl_cam = readtable('tum_dataset\dataset-calib-cam1_512_16\mav0\cam0\data.csv');
camTimeSteps = Tbl_cam.x_timestamp_ns_;
camTimeSteps = (camTimeSteps - camTimeSteps(1))*1e-9;
camImageName = Tbl_cam.filename;
camSteps     = size(Tbl_cam,1);
% Save Important Matrices
save('pqfile2.mat','camPoses','camPoses_Transl','xyzPoints','xyzPoints1','vSet','reprojectionErrors1','reprojectionErrors2')
toc
%% Display the refined camera poses and 3-D world points from Structure from Motion Algorithm
tic
if SfM
    % Display the images.
    imageDIR = fullfile('tum_dataset\dataset-calib-cam1_512_16\mav0\cam0\data');
    figure
    imds = imageDatastore(imageDIR);
    montage(imds.Files, 'Size', [5, 5]);
    title('Input Image Sequence');

    % Display camera poses.
    camPosess = poses(vSet);
    figure;
    plotCamera(camPosess, 'Size', 0.2);
    hold on

    % Exclude noisy 3-D points.
    % goodIdx = (reprojectionErrors1 < 15);
    xyzPoints1 = xyzPoints1(:, :);

    % Display the 3-D points.
    pcshow(xyzPoints1, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
        'MarkerSize', 45);
    grid on
    hold off

    % Specify the viewing volume.
    loc1 = camPosess.Location{1};
    xlim([loc1(1)-5, loc1(1)+4]);
    ylim([loc1(2)-5, loc1(2)+4]);
    zlim([loc1(3)-1, loc1(3)+20]);
    camorbit(0, -30);
    title('Refined Camera Poses for all key-frames');

    % Display the dense point cloud -Dense Reconstruction-
    % Display the refined camera poses.
    figure;
    % Exclude noisy 3-D world points.
    % goodIdx = (reprojectionErrors2 < 5);

    % Display the dense 3-D world points.
    pcshow(xyzPoints(:, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
        'MarkerSize', 45);
    grid on

    % Specify the viewing volume.
    loc1 = camPosess.Location{1};
    xlim([loc1(1)-5, loc1(1)+4]);
    ylim([loc1(2)-5, loc1(2)+4]);
    zlim([loc1(3)-1, loc1(3)+20]);
    camorbit(0, -30);
    title('3D-Scene Reconstruction for all key-frames');
else 
    disp('Please, run or load the Structure from Motion Algorithm!')
end
toc
%% IMU Sensors
% Typically, a UAV uses an integrated MARG sensor (Magnetic, Angular Rate,
% Gravity) for pose estimation. To model a MARG sensor, define an IMU
% sensor model containing an accelerometer and gyroscope. In
% a real-world application the two sensors could come from a single
% integrated circuit or separate ones. The property values set here are
% typical for low-cost MEMS sensors.
tic
% IMU Data Reading
Tbl = readtable('tum_dataset\dataset-calib-cam1_512_16\mav0\imu0\data1.csv');
imuA.data.gyro  = [Tbl.w_RS_S_x_radS__1_,Tbl.w_RS_S_y_radS__1_,Tbl.w_RS_S_z_radS__1_];
imuA.data.accel = [Tbl.a_RS_S_x_mS__2_,Tbl.a_RS_S_y_mS__2_,Tbl.a_RS_S_z_mS__2_];
imuA.data.time  = Tbl.x_timestamp_ns_;
imuA.data.time  = (imuA.data.time - imuA.data.time(1))*1e-9; 

imuSteps = size(Tbl,1);

[q_imu]=imu_quat(imuA.data.accel(:,1),imuA.data.accel(:,2),imuA.data.accel(:,3),...
                imuA.data.gyro(:,1),imuA.data.gyro(:,2),imuA.data.gyro(:,3),imuA.data.time);
q_imu = q_imu';   
imuOrientations = quat2eul(q_imu,'ZYX')*180/pi; % In Degrees

accelerometer_random_walk   =  (0.00086^2*T)*ones(1,3);   % sigma_nba->[m/s^3/sqrt(Hz)](accel bias diffusion)
accelerometer_noise_density =  (0.0028^2/T)*ones(1,3);    % sigma_na->[m/s^2/sqrt(Hz)](accel "white noise")
gyroscope_random_walk       =  (2.2e-05^2*T)*ones(1,3);   % sigma_nbw->[rad/s^2/sqrt(Hz)](gyro bias diffusion)
gyroscope_noise_density     =  (0.00016^2/T)*ones(1,3);   % sigma_nw->[rad/s/sqrt(Hz)](gyro "white noise")
toc
%% Initialize the State Vector of the Camera  + IMU |Extended-Kalman filter| "28 - Error States"
% The |Extended-Kalman filter| tracks the pose states in a 31-element vector.
% The states are:
% 
%          State                       Units        State Vector Index     Error State Vector Index
%   Position      (world-imu)          m            1:3                    1:3
%   Velocity      (world-imu)          m/s          4:6                    4:6
%   Angle         (world-imu)          rad          7:10                   7:9
%   Gyroscope Bias                     m/s          11:13                  10:12
%   Accelerometer Bias                 rad          14:16                  13:15
%   Visual Odometry Scale              unitless     17                     16
%   Position      (imu-cam)            m            18:20                  17:19
%   Angle         (imu-cam)            rad          21:24                  20:22
%   Position      (visual-world)       m            25:27                  23:25
%   Angle         (visual-world)       rad          28:31                  26:28
%
% We are using an error states extended kalman filter so the internal initial error states
% will all be equal to zero for accurate and fast convergance.

% Initialize the states of the filter 
tic
States    = zeros(31,1);
initstate = zeros(31,1);
initstate(1:3)   = gtPositions(1,:); 
initstate(4:6)   = 0;
initstate(7:10)  = [1,0,0,0];
initstate(11:13) = gyroscope_random_walk;
initstate(14:16) = accelerometer_random_walk;
initstate(17)    = 1;
initstate(18:20) = pi_c;
initstate(21:24) = [1;0;0;0];
initstate(25:27) = 0;
initstate(28:31) = [1;0;0;0];
fusionfilt.State      = initstate;
ErrorState = zeros(28,1);
fusionfilt.ErrorState = ErrorState;
toc
%% Initialize the Variances of the Camera  + IMU |Extended-Kalman filter|
% The |Extended-Kalman filter| measurement noises describe how much noise is
% corrupting the sensor reading. These values are based on the |imuSensor| and
% |camSensor| parameters.
%
% The process noises describe how well the filter equations describe the
% state evolution. Process noises are determined empirically using
% parameter sweeping to jointly optimize position and orientation
% estimates from the filter. 
tic
% Measurement noises (R) for Visual Monocular Odometry CAM
Rang = [0.01128,0.01185,0.01152]; % CAM Angle    measurement noise (vision-cam)
Rpos = [9.59e-8,8.98e-9,0.0000011568]; % CAM Position measurement noise (vision-cam)
dR   = [Rpos,Rang];

R = diag(dR);

% Process noises (Qc)
dQc = [accelerometer_noise_density,accelerometer_random_walk, ...
    gyroscope_noise_density,gyroscope_random_walk];

Qc = diag(dQc);

% Initial error covariance
% P = 1e-5*eye(28);
% P = zeros(28);
% P(1:3,1:3)  = 10^-7; P(4:6,4:6)  = 10^-7; P(7:9,7:9)  = 10^-5;
% P(2,2)      = 10^7; P(5,5)      = 10^-7;
% P(10:12,10:12)   = 10^-7;
% P(13:15,13:15)   = 10^-9;
% P(16,16) = 1;
% P(17:19,17:19) = 10^-7; P(20:22,20:22) = 10^-7;
% P(23:25,23:25) = 10^-3; P(26:28,26:28) = 10^-3;
P = diag([10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,10^-7,1,10^-7,10^-7,10^-7,...
          10^-7,10^-7,10^-7,10^-3,10^-3,10^-3,10^-3,10^-3,10^-3]);
fusionfilt.StateCovariance = P;
toc
%% Simulation Loop for Camera and IMU comparing with Trajectory Ground Truth including Visual Drifts
% The main simulation loop is a for loop with an if condition. The
% for loop executes at |gtFs|, which is the Ground Truth sample rate. The
% if condition executes at |camFs|, which is the CAM sample rate. The
% scopes are updated at the IMU sample rate.
tic
% Loop setup - |imu_Ground_Truth_Data| 
framesToSimulate = imuSteps; % simulate about times of IMU 108 seconds

% Log data for final metric computation.
pqorient = quaternion.zeros(framesToSimulate,1);
pqpos    = zeros(framesToSimulate,3);
pqvel    = zeros(framesToSimulate,3);

pqorient(1,1) = gtQuaternions(1);
pqpos(1,:)    = gtPositions(1,:);
pqvel(1,:)    = fusionfilt.State(4:6)'; 

lmda     = zeros(framesToSimulate,1);
bias_a   = zeros(framesToSimulate,3);
bias_w   = zeros(framesToSimulate,3);
Errors   = zeros(28,camSteps);

test_estimated  = quaternion.zeros(camSteps,1);
test_qvw        = quaternion.zeros(camSteps,1);
test_pi_c       = zeros(framesToSimulate,3);
test_pestimated = zeros(camSteps,3);
test_pestimated(1,:) = gtPositions(1,:);

if useVO
    camPoses_Transl(1,:) = gtPositions(1,:);
    camPoses_Orient(1,:) = quat2eul (gtQuaternions(1), 'ZYX' )*180/pi;
end

lmda(1,1)     = fusionfilt.State(17);
bias_w(1,:)   = fusionfilt.State(11:13)';
bias_a(1,:)   = fusionfilt.State(14:16)';
Errors(:,1)   = fusionfilt.ErrorState;

k     = 1;
k_cam = 2;
k_fp  = 1; 
fp_limit = 2;
qvw_fp   = zeros(fp_limit,4);
qw_i     = q_imu(1,:);

camRead = true;
imuRead = true;
ff = 2;

while imuRead
    
    ErrorStates = fusionfilt.ErrorState;
    States = fusionfilt.State;
    P      = fusionfilt.StateCovariance;
    
    if camRead
        distt  = abs(camTimeSteps(k_cam)-imuA.data.time);
        k_imu  = find(distt == min(distt));
        distat  = abs(camTimeSteps(k_cam)-gtTimeSteps);
        k_gt  = find(distat == min(distat));
    end
      
    % Simulate the IMU data from the current & previous poses.(Without Noise For Estimation)
    accel.curr  = imuA.data.accel(ff,:)-States(14:16)';         % 1x3 Accelerations (ZYX)
    accel.prev  = imuA.data.accel(ff-1,:);       % 1x3 Accelerations (ZYX)
    gyro.curr   = imuA.data.gyro(ff,:)-States(11:13)';    % 1x3 Angular Velocities (ZYX)
    gyro.prev   = imuA.data.gyro(ff-1,:);  % 1x3 Angular Velocities (ZYX)
    gyro.avrg   = (gyro.curr+gyro.prev)/2;
    
    % Current Accelerometer Readings (With Noise For Measurements)
    am_curr = accel.curr';   % Current Accel. Readings + delta_ba
    am_skew_curr = [0,-am_curr(3),am_curr(2);am_curr(3),0,-am_curr(1);-am_curr(2),am_curr(1),0];
        
    wm_curr = gyro.curr';    % Current Gyro. Readings + delta_bw
    wmskew_curr = [0,-wm_curr(3),wm_curr(2);wm_curr(3),0,-wm_curr(1);-wm_curr(2),wm_curr(1),0]; 
    
    % Using the current data (t)
    a_skew_estimated = am_skew_curr;
    wskew_estimated  = wmskew_curr;   
        
    % Step : 1 -> States Propagation for the current time step 
    [States,qw_i] = IMU_Propagation_(States,T,accel,gyro,qw_i);   % Paper IMU States Propagation Formulas -Recommended- 
%     [States,qw_i] = IMU_Propagation(States,T,accel.curr,gyro,qw_i);   % MATLAB IMU States Propagation Formulas
    Cqw_i = quat2rotm(qw_i); % Rotation Matrix of World Coord. wrt. IMU Coord.

    % Step : 2 -> States Covariance Estimation [[P]] (Noise Considered)
    P = get_P(T,Qc,Cqw_i,a_skew_estimated,wskew_estimated,P);
       
    if ff == k_imu 
        % Simulate the CAM data from the current pose.
        if useVO 
            [transl, orient, paramsVO] = helperVisualOdometryModel(gtPositions(k_gt,:),quat2eul(gtQuaternions(k_gt),'ZYX'), paramsVO);
            camPoses_Transl(k_cam,:) = transl;
            camPoses_Orient(k_cam,:) = rotm2eul(orient,'ZYX')*180/pi;
            
        else
            orient = double(cell2mat(camPoses(k_cam,1))); % 3x3 Rotation Matrix (qv_c)
            transl = camPoses_Transl(k_cam,:);            % 1x3 Linear Translations Vector (pv_c)
        end
        
        % Fuse CAM & IMU to get the current estimate of the EK-filtered states.
        % Current Camera Readings (Sfm Vision Algorithm Output)
        mpv_c = transl;                   % pv_c
        mqv_c = rotm2quat (orient);       % qv_c
        
        % False Pose Estimation Detection
        qv_w  = compact(quaternion(quatinv(qw_i))*quaternion(quatinv(States(21:24)'))*quaternion(mqv_c));
        qv_w  = quaternion(qv_w/quatnorm(qv_w));
%         qvw_fp(k_fp,:) = compact(qv_w);
%         if k_fp == fp_limit
%             k_fp     = 0;
%             [Qvw_fp] = median_filter_fp(qvw_fp);
%             qv_w     = quaternion(Qvw_fp);
%         end
        if k_cam == 2
            Cqv_w = quat2rotm(qv_w);  % Rotation Matrix of Vision Coord. wrt. World Coord.
        end
        test_qvw(k_cam,1) = quaternion(qv_w);
        
        % Step : 3-> Measurement Model [[H]] and the Residual [[Z_tilda]]    
        Zp_tilda = mpv_c' - (Cqv_w'*(States(1:3)+Cqw_i'*States(18:20))*States(17));
        zq_tilda = quaternion(mqv_c)*quaternion(quatinv(compact((quaternion(States(21:24)')*quaternion(qw_i)*qv_w))));
        Zq_tilda = compact(zq_tilda);
        Z_tilda  = [Zp_tilda;quat2eul(Zq_tilda,'ZYX')'];
        test_estimated(k_cam,1)  = quaternion(States(21:24)')*quaternion(qw_i)*qv_w;
        test_pestimated(k_cam,:) = (Cqv_w'*(States(1:3)+Cqw_i'*States(18:20))*States(17));
        
        v1 = States(18:20);
        v1_skew = [0,-v1(3),v1(2);v1(3),0,-v1(1);-v1(2),v1(1),0];
        v2 = (States(1:3)+Cqw_i'*States(18:20))*States(17);
        v2_skew = [0,-v2(3),v2(2);v2(3),0,-v2(1);-v2(2),v2(1),0];
                
        Hp = [Cqv_w'*States(17),zeros(3),-Cqv_w'*Cqw_i'*v1_skew*States(17),zeros(3),...
                zeros(3),(Cqv_w'*Cqw_i'*v1+Cqv_w'*States(1:3)),...
                    Cqv_w'*Cqw_i'*States(17),zeros(3),zeros(3),-Cqv_w'*v2_skew];

        if k_cam == 2
            Ew_i = quat2eul(qw_i,'ZYX');
            Ei_c = quat2eul(qi_c,'ZYX');
            Ev_w = quat2eul(compact(qv_w),'ZYX');
        end
        
        ew_i   = Ew_i;
        d_qw_i = [0,-ew_i(3),ew_i(2);ew_i(3),0,-ew_i(1);-ew_i(2),ew_i(1),0];
        n_qw_i = norm(ew_i);
        ei_c   = Ei_c;
        d_qi_c = [0,-ei_c(3),ei_c(2);ei_c(3),0,-ei_c(1);-ei_c(2),ei_c(1),0];
        n_qi_c = norm(ei_c);
        ev_w   = Ev_w;
        d_qv_w = [0,-ev_w(3),ev_w(2);ev_w(3),0,-ev_w(1);-ev_w(2),ev_w(1),0];
        n_qv_w = norm(ev_w);
        
        h_qw_i = eye(3)-((1-cos(n_qw_i))/n_qw_i^2)*d_qw_i+((n_qw_i-sin(n_qw_i))/n_qw_i^3)*d_qw_i^2;
        h_qi_c = eye(3)-((1-cos(n_qi_c))/n_qi_c^2)*d_qi_c+((n_qi_c-sin(n_qi_c))/n_qi_c^3)*d_qi_c^2;
        h_qv_w = eye(3)-((1-cos(n_qv_w))/n_qv_w^2)*d_qv_w+((n_qv_w-sin(n_qv_w))/n_qv_w^3)*d_qv_w^2;
              
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
        States(1:3)   = States(1:3)   + ErrorStates(1:3);
        States(4:6)   = States(4:6)   + ErrorStates(4:6);
        States(11:17) = States(11:17) + ErrorStates(10:16);
        States(18:20) = States(18:20) + ErrorStates(17:19);
        States(25:27) = States(25:27) + ErrorStates(23:25);   
        
        dqw_i = qw_i'+0.5*[qw_i(1),-qw_i(3),qw_i(2);qw_i(3),qw_i(1),-qw_i(4);-qw_i(2),qw_i(4),qw_i(1);-qw_i(4),-qw_i(2),-qw_i(3)]*ErrorStates(7:9);%quatmultiply(dqw_i,qw_i);
        dqw_i = dqw_i';
        Qi_c  = States(21:24);
        dqi_c = Qi_c+0.5*[Qi_c(1),-Qi_c(3),Qi_c(2);Qi_c(3),Qi_c(1),-Qi_c(4);-Qi_c(2),Qi_c(4),Qi_c(1);-Qi_c(4),-Qi_c(2),-Qi_c(3)]*ErrorStates(20:22);%quatmultiply(dqi_c,States(21:24)');
        dqi_c = dqi_c';
        Qv_w  = compact(qv_w)';
        dqv_w = Qv_w+0.5*[Qv_w(1),-Qv_w(3),Qv_w(2);Qv_w(3),Qv_w(1),-Qv_w(4);-Qv_w(2),Qv_w(4),Qv_w(1);-Qv_w(4),-Qv_w(2),-Qv_w(3)]*ErrorStates(26:28);%quatmultiply(dqv_w,compact(qv_w));
        dqv_w = dqv_w';
        
        States(7:10)  = dqw_i/quatnorm(dqw_i);  % Estimated^qw_i compact(test_estimated(k_cam,1))/quatnorm(compact(test_estimated(k_cam,1)));
        qw_i  = dqw_i/quatnorm(dqw_i);
        qw_ix = [0,-ErrorStates(9),ErrorStates(8);ErrorStates(9),0,-ErrorStates(7);-ErrorStates(8),ErrorStates(7),0];
        Cqw_i = (eye(3)+qw_ix)*Cqw_i;
        
        States(21:24) = dqi_c/quatnorm(dqi_c); % Estimated^qi_c
        qi_c  = dqi_c/quatnorm(dqi_c);
        qi_cx = [0,-ErrorStates(22),ErrorStates(21);ErrorStates(22),0,-ErrorStates(20);-ErrorStates(21),ErrorStates(20),0];
        Cqi_c = (eye(3)+qi_cx)*Cqi_c;
        
        States(28:31) = dqv_w/quatnorm(dqv_w); % Estimated^qv_w
        qv_w  = quaternion(dqv_w/quatnorm(dqv_w));
        qv_wx = [0,-ErrorStates(28),ErrorStates(27);ErrorStates(28),0,-ErrorStates(26);-ErrorStates(27),ErrorStates(26),0];
        Cqv_w = (eye(3)+qv_wx)*Cqv_w;
        
        % ESKF Covariance Matrix RESET
        G = [eye(6),zeros(6,22);zeros(3,6),eye(3)-0.5*qw_ix,zeros(3,19);zeros(10,9),eye(10),zeros(10,9);zeros(3,19),...
            eye(3)-0.5*qi_cx,zeros(3,6);zeros(3,22),eye(3),zeros(3);zeros(3,25),eye(3)-0.5*qv_wx];
        P = G*P*G';
        
        % ESKF Orientation error RESET
        qw_ix = [0,-ErrorStates(9),ErrorStates(8);ErrorStates(9),0,-ErrorStates(7);-ErrorStates(8),ErrorStates(7),0];
        Ew_i  = -ErrorStates(7:9)+(eye(3)-0.5*qw_ix)*Errors(7:9,k_cam-1);
        qi_cx = [0,-ErrorStates(22),ErrorStates(21);ErrorStates(22),0,-ErrorStates(20);-ErrorStates(21),ErrorStates(20),0];
        Ei_c  = -ErrorStates(20:22)+(eye(3)-0.5*qi_cx)*Errors(20:22,k_cam-1);
        qv_wx = [0,-ErrorStates(28),ErrorStates(27);ErrorStates(28),0,-ErrorStates(26);-ErrorStates(27),ErrorStates(26),0];
        Ev_w  = -ErrorStates(26:28)+(eye(3)-0.5*qv_wx)*Errors(26:28,k_cam-1);   
        
        % Save Errors for post processing.
        Errors(:,k_cam) = ErrorStates;
        fusionfilt.ErrorState = ErrorStates;
        
        disp("Fusion for this Camera frame done Successfully!");

        k_cam = k_cam + 1;
        k_fp  = k_fp + 1;
        if k_cam > camSteps
            camRead = false;
        end
    end
    % Update the Fusion Filter
    fusionfilt.StateCovariance = P;
    fusionfilt.State           = States;
                   
    % Save the Accel., Gyro Biases, scale factor, position and orientation for post processing.
    pqpos(ff,:)     = fusionfilt.State(1:3)';
    pqvel(ff,:)     = fusionfilt.State(4:6)';
    pqorient(ff,1)  = quaternion(fusionfilt.State(7:10)');
    lmda(ff,1)      = fusionfilt.State(17);
    bias_w(ff,:)    = fusionfilt.State(11:13)';
    bias_a(ff,:)    = fusionfilt.State(14:16)';
    test_pi_c(ff,:) = fusionfilt.State(18:20)';
    
    ff = ff + 1;
    disp(["Processing Current IMU Data!",num2str(100*ff/framesToSimulate),"%"]);
    if ff >= imuSteps
        imuRead = false;
    end

end

% Orientations for plotting in Degrees (Roll - Pitch - Yaw) from Estimated Quaternions
euler_orient = quat2eul(pqorient,'ZYX')*180/pi;
fuseTimeSteps = imuA.data.time(1:ff);
disp("Simulation for all time steps done Successfully!");
toc
%% Error Metric Computation
% Position and orientation estimates were logged throughout the
% simulation. Now compute an end-to-end root mean squared error for both
% position and orientation.
% For orientation, quaternion distance is a much better alternative to
% subtracting Euler angles, which have discontinuities. The quaternion
% distance can be computed with the |dist| function, which gives the
% angular difference in orientation in radians. Convert to degrees
% for display in the command window. 
tic
k_gt = 1;
for f=1:framesToSimulate
    disttt  = abs(gtTimeSteps(k_gt)-imuA.data.time);
    k_imu = find(disttt == min(disttt));
    if f == k_imu
        posd(k_gt,:) = pqpos(f,:) - gtPositions(k_gt,:);
        quatd(k_gt)  = rad2deg(dist(pqorient(f,1), gtQuaternions(k_gt,1)) );
        k_gt = k_gt + 1;
        if k_gt > gtSteps
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
toc
%% Testing Quaternions and Positions Estimated
tic
test_orient = quat2eul(test_estimated,'ZYX')*180/pi;
test_qvwDeg = quat2eul(test_qvw,'ZYX')*180/pi;

figure
plot(camTimeSteps,test_qvwDeg(:,1),camTimeSteps,test_qvwDeg(:,2),camTimeSteps,test_qvwDeg(:,3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('attitude drift q_v^w [deg]')
title('Roll, pitch, yaw representation of the calculated rotation between the black-box visual frame and the world frame')
legend('\phi roll','\theta pitch','\psi yaw')

figure
plot(camTimeSteps,test_qvwDeg(:,1),fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),1),gtTimeSteps,gtOrientations(:,1))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('\phi roll [deg]')
title('Example for the roll angle estimate evolution during a failure of the black-box visual framework')
legend('Visual Measurement','EKF-Fusion','Ground-Truth')

figure
plot(fuseTimeSteps,test_pi_c(1:length(fuseTimeSteps),1),fuseTimeSteps,test_pi_c(1:length(fuseTimeSteps),2),fuseTimeSteps,test_pi_c(1:length(fuseTimeSteps),3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('P_i^c (m)')
title('Fused IMU & CAM Positions in X,Y,Z')
legend('P_i^c X','P_i^c Y','P_i^c Z')

figure
plot(camTimeSteps,test_pestimated(:,1),camTimeSteps,camPoses_Transl(:,1),fuseTimeSteps,pqpos(1:length(fuseTimeSteps),1),gtTimeSteps,gtPositions(:,1))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('X (m)')
title('Fused IMU & CAM (x-position) with Ground-Truth')
legend('Est-CAM X','CAM-10 KF','EKF-Fusion','Ground-Truth')

figure
plot(camTimeSteps,test_pestimated(:,2),camTimeSteps,camPoses_Transl(:,2),fuseTimeSteps,pqpos(1:length(fuseTimeSteps),2),gtTimeSteps,gtPositions(:,2))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('Y (m)')
title('Fused IMU & CAM (y-position) with Ground-Truth')
legend('Est-CAM Y','CAM-10 KF','EKF-Fusion','Ground-Truth')

figure
plot(camTimeSteps,test_pestimated(:,3),camTimeSteps,camPoses_Transl(:,3),fuseTimeSteps,pqpos(1:length(fuseTimeSteps),3),gtTimeSteps,gtPositions(:,3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('Z (m)')
title('Fused IMU & CAM (z-position) with Ground-Truth')
legend('Est-CAM Z','CAM-10 KF','EKF-Fusion','Ground-Truth')

figure
plot(camTimeSteps,test_orient(:,1),camTimeSteps,camPoses_Orient(:,1),fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),1),gtTimeSteps,gtOrientations(:,1))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('\phi°')
title('Fused IMU & CAM (Roll-Orientation) with Ground-Truth')
legend('Est-CAM \phi','CAM-10 KF','EKF-Fusion','Ground-Truth')

figure
plot(camTimeSteps,test_orient(:,2),camTimeSteps,camPoses_Orient(:,2),fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),2),gtTimeSteps,gtOrientations(:,2))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('\theta°')
title('Fused IMU & CAM (Pitch-Orientation) with Ground-Truth')
legend('Est-CAM \theta','CAM-10 KF','EKF-Fusion','Ground-Truth')

figure
plot(camTimeSteps,test_orient(:,3),camTimeSteps,camPoses_Orient(:,3),fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),3),gtTimeSteps,gtOrientations(:,3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('\psi°')
title('Fused IMU & CAM (Yaw-Orientation) with Ground-Truth')
legend('Est-CAM \psi','CAM-10 KF','EKF-Fusion','Ground-Truth')
toc
%% Results Simulation Scopes for CAM and IMU Fusion comparing with Trajectory Ground Truth including Visual Drifts
% The scopes are plotted at the IMU sample rate.
tic
% plotting and comparing the GT & CAM (Roll-Orientation)
figure
plot(gtTimeSteps,gtOrientations(:,1),camTimeSteps,camPoses_Orient(:,1))
grid on
xlabel('GT & CAM Instances (sec.)')
ylabel('\phi°')
title('CAM (Roll-Orientation) with Ground-Truth')
legend('Ground-Truth','CAM-10 KF')

% plotting and comparing the GT & CAM (Pitch-Orientation)
figure
plot(gtTimeSteps,gtOrientations(:,2),camTimeSteps,camPoses_Orient(:,2))
grid on
xlabel('GT & CAM Instances (sec.)')
ylabel('\theta°')
title('CAM (Pitch-Orientation) with Ground-Truth')
legend('Ground-Truth','CAM-10 KF')

% plotting and comparing the GT & CAM (Yaw-Orientation)
figure
plot(gtTimeSteps,gtOrientations(:,3),camTimeSteps,camPoses_Orient(:,3))
grid on
xlabel('GT & CAM Instances (sec.)')
ylabel('\psi°')
title('CAM (Yaw-Orientation) with Ground-Truth')
legend('Ground-Truth','CAM-10 KF')

% plotting and comparing the GT & CAM (x-position)
figure
plot(gtTimeSteps,gtPositions(:,1),camTimeSteps,camPoses_Transl(:,1))
grid on
xlabel('GT & CAM Instances (sec.)')
ylabel('X (m)')
title('CAM (x-position) with Ground-Truth')
legend('Ground-Truth','CAM-10 KF')

% plotting and comparing the GT & CAM (y-position)
figure
plot(gtTimeSteps,gtPositions(:,2),camTimeSteps,camPoses_Transl(:,2))
grid on
xlabel('GT & CAM Instances (sec.)')
ylabel('Y (m)')
title('CAM (y-position) with Ground-Truth')
legend('Ground-Truth','CAM-10 KF')

% plotting and comparing the GT & CAM (z-position)
figure
plot(gtTimeSteps,gtPositions(:,3),camTimeSteps,camPoses_Transl(:,3))
grid on
xlabel('GT & CAM Instances (sec.)')
ylabel('Z (m)')
title('CAM (z-position) with Ground-Truth')
legend('Ground-Truth','CAM-10 KF')

% plotting and comparing the Fused, GT & CAM (x-position) 
figure
plot(fuseTimeSteps,pqpos(1:length(fuseTimeSteps),1),gtTimeSteps,gtPositions(:,1),camTimeSteps,camPoses_Transl(:,1))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('X (m)')
title('Fused IMU & CAM (x-position) with Ground-Truth')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

% plotting and comparing the Fused, GT & CAM (y-position) 
figure
plot(fuseTimeSteps,pqpos(1:length(fuseTimeSteps),2),gtTimeSteps,gtPositions(:,2),camTimeSteps,camPoses_Transl(:,2))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('Y (m)')
title('Fused IMU & CAM (y-position) with Ground-Truth')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

% plotting and comparing the Fused, GT & CAM (z-position) 
figure
plot(fuseTimeSteps,pqpos(1:length(fuseTimeSteps),3),gtTimeSteps,gtPositions(:,3),camTimeSteps,camPoses_Transl(:,3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('Z (m)')
title('Fused IMU & CAM (z-position) with Ground-Truth')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

% plotting and comparing the Fused, GT & CAM (xyz-velocities) 
figure
plot(fuseTimeSteps,pqvel(1:length(fuseTimeSteps),1),fuseTimeSteps,pqvel(1:length(fuseTimeSteps),2),fuseTimeSteps,pqvel(1:length(fuseTimeSteps),3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('Velocity (m / sec.)')
title('Fused IMU & CAM Velocities')
legend('Velocity in X','Velocity in Y','Velocity in Z')

% plotting and comparing the IMU & Ground-Truth (Roll) 
figure
plot(imuA.data.time,imuOrientations(:,1),gtTimeSteps,gtOrientations(:,1),fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),1));
grid on
xlabel('IMU & Ground-Truth Instances (sec.)')
ylabel('\phi°')
title('Comparing Fused with the IMU & Ground-Truth (Roll)')
legend('IMU','Ground-Truth','EKF-Fusion')

% plotting and comparing the IMU & Ground-Truth (Pitch) 
figure
plot(imuA.data.time,imuOrientations(:,2),gtTimeSteps,gtOrientations(:,2),fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),2));
grid on
xlabel('IMU & Ground-Truth Instances (sec.)')
ylabel('\theta°')
title('Comparing Fused with the IMU & Ground-Truth (Pitch)')
legend('IMU','Ground-Truth','EKF-Fusion')

% plotting and comparing the IMU & Ground-Truth (Yaw) 
figure
plot(imuA.data.time,imuOrientations(:,3),gtTimeSteps,gtOrientations(:,3),fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),3));
grid on
xlabel('IMU & Ground-Truth Instances (sec.)')
ylabel('\psi°')
title('Comparing Fused with the IMU & Ground-Truth (Yaw)')
legend('IMU','Ground-Truth','EKF-Fusion')

% plotting and comparing the CAM, GT & Fused (phi-orientation-Roll)
figure
plot(fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),1),gtTimeSteps,gtOrientations(:,1),camTimeSteps,camPoses_Orient(:,1))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('\phi°')
title('Fused IMU & CAM Roll(\phi)')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

% plotting and comparing the CAM, GT & Fused (theta-orientation-Pitch)
figure
plot(fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),2),gtTimeSteps,gtOrientations(:,2),camTimeSteps,camPoses_Orient(:,2))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('\theta°')
title('Fused IMU & CAM Pitch(\theta)')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

% plotting and comparing the CAM, GT & Fused (psi-orientation-Yaw)
figure
plot(fuseTimeSteps,euler_orient(1:length(fuseTimeSteps),3),gtTimeSteps,gtOrientations(:,3),camTimeSteps,camPoses_Orient(:,3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('\psi°')
title('Fused IMU & CAM Yaw(\psi)')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

% Additional Scopes 
% Errors Plots between fused and ground truth values
figure
plot(camTimeSteps,Errors(1,:),camTimeSteps,Errors(2,:),camTimeSteps,Errors(3,:))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('Error (m)')
title('Fused IMU & CAM Errors in XYZ-Positions')
legend('Error in x-pos.','Error in y-pos.','Error in z-pos.')

figure
plot(camTimeSteps,Errors(4,:),camTimeSteps,Errors(5,:),camTimeSteps,Errors(6,:))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('Error (m / sec.)')
title('Fused IMU & CAM Errors in XYZ-Velocities')
legend('Error in x-vel.','Error in y-vel.','Error in z-vel.')

figure
plot(camTimeSteps,Errors(7,:),camTimeSteps,Errors(8,:),camTimeSteps,Errors(9,:))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('Error (rad.)')
title('Fused IMU & CAM Errors in ZYX-Orientations')
legend('Error in \phi','Error in \theta','Error in \psi')

% (biases - Visual Odometry scale)
figure
plot(fuseTimeSteps,bias_a(1:length(fuseTimeSteps),1),fuseTimeSteps,bias_a(1:length(fuseTimeSteps),2),fuseTimeSteps,bias_a(1:length(fuseTimeSteps),3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('b_a')
title('Accelerometer Biases')
legend('b_a in x-direct.','b_a in y-direct.','b_a in z-direct.')

figure
plot(fuseTimeSteps,bias_w(1:length(fuseTimeSteps),1),fuseTimeSteps,bias_w(1:length(fuseTimeSteps),2),fuseTimeSteps,bias_w(1:length(fuseTimeSteps),3))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('b_w')
title('Gyroscope Biases')
legend('b_w in x-direct.','b_w in y-direct.','b_w in z-direct.')

figure
plot(fuseTimeSteps,lmda(1:length(fuseTimeSteps),1))
grid on
xlabel('Fused IMU & CAM Instances (sec.)')
ylabel('\lambda')
title('Visual Monocular Odometry Scale Factor')

% Paths Comparison Plots
figure
plot3(pqpos(1:length(fuseTimeSteps),1),pqpos(1:length(fuseTimeSteps),2),pqpos(1:length(fuseTimeSteps),3))
hold all
plot3(gtPositions(:,1),gtPositions(:,2),gtPositions(:,3))
hold all
plot3(camPoses_Transl(:,1),camPoses_Transl(:,2),camPoses_Transl(:,3))
grid on
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
title('Comparing Ground Truth and Fused IMU & CAM 3D - Path')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

figure
plot3(euler_orient(1:length(fuseTimeSteps),1),euler_orient(1:length(fuseTimeSteps),2),euler_orient(1:length(fuseTimeSteps),3))
hold all
plot3(gtOrientations(:,1),gtOrientations(:,2),gtOrientations(:,3))
hold all
plot3(camPoses_Orient(:,1),camPoses_Orient(:,2),camPoses_Orient(:,3))
grid on
xlabel('Roll (\phi°)')
ylabel('Pitch (\theta°)')
zlabel('Yaw (\psi°)')
title('Comparing Ground Truth and Fused IMU & CAM 3D - Orientations')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

figure
plot(pqpos(1:length(fuseTimeSteps),1),pqpos(1:length(fuseTimeSteps),2))
hold all
plot(gtPositions(:,1),gtPositions(:,2))
hold all
plot(camPoses_Transl(:,1),camPoses_Transl(:,2))
grid on
xlabel('X (m)')
ylabel('Y (m)')
title('Comparing Ground Truth and Fused IMU & CAM X-Y 2D - Path')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

figure
plot(pqpos(1:length(fuseTimeSteps),1),pqpos(1:length(fuseTimeSteps),3))
hold all
plot(gtPositions(:,1),gtPositions(:,3))
hold all
plot(camPoses_Transl(:,1),camPoses_Transl(:,3))
grid on
xlabel('X (m)')
ylabel('Z (m)')
title('Comparing Ground Truth and Fused IMU & CAM X-Z 2D - Path')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

figure
plot(pqpos(1:length(fuseTimeSteps),2),pqpos(1:length(fuseTimeSteps),3))
hold all
plot(gtPositions(:,2),gtPositions(:,3))
hold all
plot(camPoses_Transl(:,2),camPoses_Transl(:,3))
grid on
xlabel('Y (m)')
ylabel('Z (m)')
title('Comparing Ground Truth and Fused IMU & CAM Y-Z 2D - Path')
legend('EKF-Fusion','Ground-Truth','CAM-10 KF')

% Reinitialization Plot (influence of a map-loss to the visual drift states)


toc