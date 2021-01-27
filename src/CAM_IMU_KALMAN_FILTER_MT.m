clc;clear all;close all;
load('loggedGroundVehicleCircle.mat', ...
    'imuFs','localOrigin', ...
    'initialStateCovariance', ...
    'accelData','gyroData', ...
    'gpsFs','gpsLLA','Rpos','gpsVel','Rvel', ...
    'trueOrient','truePos');
initialState = [compact(trueOrient(1)),truePos(1,:),-6.8e-3,2.5002,0,zeros(1,6),1].';
filt = insfilterErrorState;
filt.IMUSampleRate = imuFs;
filt.ReferenceLocation = localOrigin;
filt.State = initialState;
filt.StateCovariance = initialStateCovariance;
numIMUSamples = size(accelData,1);
estOrient = ones(numIMUSamples,1,'quaternion');
estPos = zeros(numIMUSamples,3);
gpsIdx = 1;
for idx = 1:numIMUSamples

    % Use predict to estimate the filter state based on the accelData and
    % gyroData arrays.
    predict(filt,accelData(idx,:),gyroData(idx,:));
    
    % GPS data is collected at a lower sample rate than IMU data. Fuse GPS
    % data at the lower rate.
    if mod(idx, imuFs / gpsFs) == 0
        % Correct the filter states based on the GPS data.
        fusegps(filt,gpsLLA(gpsIdx,:),Rpos,gpsVel(gpsIdx,:),Rvel);
        gpsIdx = gpsIdx + 1;
    end
    
    % Log the current pose estimate
    [estPos(idx,:), estOrient(idx,:)] = pose(filt);
end
pErr = truePos - estPos;
qErr = rad2deg(dist(estOrient,trueOrient));

pRMS = sqrt(mean(pErr.^2));
qRMS = sqrt(mean(qErr.^2));

fprintf('Position RMS Error\n');
fprintf('\tX: %.2f, Y: %.2f, Z: %.2f (meters)\n\n',pRMS(1),pRMS(2),pRMS(3));

fprintf('Quaternion Distance RMS Error\n');
fprintf('\t%.2f (degrees)\n\n',qRMS);
plot(truePos(:,1),truePos(:,2),estPos(:,1),estPos(:,2),'r:','LineWidth',2)
grid on
axis square
xlabel('N (m)')
ylabel('E (m)')
legend('Ground Truth','Estimation')