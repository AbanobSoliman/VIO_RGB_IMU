%% Developed by Abanob Soliman for Master Thesis in Computer Vision and Sensor Fusion 
% under the Supervision of Prof. Samia Bouchafa Head of IBISC Laboratory, University of Evry, France
%% Structure From Motion Algorithm in order to obtain the Camera Pose and the 3D Scene Construction
clc;clear all;close all;
%% DataSet Initialization
% Use |imageDatastore| to get a list of all image file names in a directory.

% Activate Next Line For DEMO Model
% imageDir = fullfile(toolboxdir('vision'), 'visiondata', 'structureFromMotion');

% Activate any one of the next 2 Lines For REAL Model
imageDir = fullfile('cam_april\mav0\cam0\data\filtered');

imds = imageDatastore(imageDir);

% Display the images.
figure
% montage(imds.Files, 'Size', [1, 5]);
montage(imds.Files, 'Size', [3, 6]);

% Convert the images to grayscale.
% Get the number of rows and columns, most importantly, the number of color channels.
images = cell(1, numel(imds.Files));
[rows, columns, numberOfColorChannels] = size(readimage(imds, 1));
if numberOfColorChannels > 1
    % It's a true color RGB image.  We need to convert to gray scale.
    for i = 1:numel(imds.Files)
        I = readimage(imds, i);
        images{i} = rgb2gray(I);
    end
else
    % It's already gray scale.  No need to convert.
    for i = 1:numel(imds.Files)
        I = readimage(imds, i);
        images{i} = I;
    end
end
title('Input Image Sequence');

% Load the cameraParameters object created using the Camera Calibrator app
% Activate Next 2 Line For DEMO Model
% data = load(fullfile(imageDir, 'cameraParams.mat'));
% cameraParams = data.cameraParams;
% Activate Next Line For REAL Model
%cameraParams = cameraParameters('IntrinsicMatrix',[458.654,0,0;0, 457.296,0; 367.215, 248.375,1],'RadialDistortion',[-0.28340811, 0.07395907],'TangentialDistortion',[0.00019359, 1.76187114e-05],'RotationVectors',[0.0148655429818, -0.999880929698, 0.00414029679422; 0.999557249008, 0.0149672133247, 0.025715529948;-0.0257744366974, 0.00375618835797, 0.999660727178],'TranslationVectors',[-0.0216401454975,-0.064676986768,0.00981073058949;-0.0216401454975,-0.064676986768,0.00981073058949;-0.0216401454975,-0.064676986768,0.00981073058949]);
cameraParams = cameraParameters('IntrinsicMatrix',[458.654,0,0;0, 457.296,0; 367.215, 248.375,1],'RadialDistortion',[-0.28340811, 0.07395907],'TangentialDistortion',[0.00019359, 1.76187114e-05],'RotationVectors',[0.0148655429818, -0.999880929698, 0.00414029679422; 0.999557249008, 0.0149672133247, 0.025715529948;-0.0257744366974, 0.00375618835797, 0.999660727178],'TranslationVectors',[0,-0.00981073058949,-0.064676986768;0.00981073058949,0,0.0216401454975;0.064676986768,-0.0216401454975,0]);
%% Create a View Set Containing the First View - Detecting Features (SURF)
% Use a viewSet object to store and manage the image points and the camera pose associated with each view, as well as point matches between pairs of views. Once you populate a viewSet object, you can use it to find point tracks across multiple views and retrieve the camera poses to be used by triangulateMultiview and bundleAdjustment functions.
% Undistort the first image.
I = undistortImage(images{1}, cameraParams); 

% Detect features. Increasing 'NumOctaves' helps detect large-scale
% features in high-resolution images. Use an ROI to eliminate spurious
% features around the edges of the image.
border = 50;
roi = [border, border, size(I, 2)- 2*border, size(I, 1)- 2*border];
prevPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);

% Extract features. Using 'Upright' features improves matching, as long as
% the camera motion involves little or no in-plane rotation.
prevFeatures = extractFeatures(I, prevPoints, 'Upright', true);

% Create an empty viewSet object to manage the data associated with each
% view.
vSet = viewSet;

% Add the first view. Place the camera associated with the first view
% and the origin, oriented along the Z-axis.
viewId = 1;
vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation', ...
eye(3, 'like', prevPoints.Location), 'Location', ...
zeros(1, 3, 'like', prevPoints.Location));
%% Add the Rest of the Views
% Go through the rest of the images. For each image: 
% 1- Match points between the previous and the current image.
% 2- Estimate the camera pose of the current view relative to the previous view.
% 3- Compute the camera pose of the current view in the global coordinate system relative to the first view.
% 4- Triangulate the initial 3-D world points.
% 5- Use bundle adjustment to refine all camera poses and the 3-D world points.
for i = 2:numel(images)
    % Undistort the current image.
    I = undistortImage(images{i}, cameraParams);
    
    % Detect, extract and match features.
    currPoints   = detectSURFFeatures(I, 'NumOctaves', 8, 'ROI', roi);
    currFeatures = extractFeatures(I, currPoints, 'Upright', true);    
    indexPairs = matchFeatures(prevFeatures, currFeatures, ...
        'MaxRatio', .7, 'Unique',  true);
    
    % Select matched points.
    matchedPoints1 = prevPoints(indexPairs(:, 1));
    matchedPoints2 = currPoints(indexPairs(:, 2));
    
    % Estimate the camera pose of current view relative to the previous view.
    % The pose is computed up to scale, meaning that the distance between
    % the cameras in the previous view and the current view is set to 1.
    % This will be corrected by the bundle adjustment.
    [relativeOrient, relativeLoc, inlierIdx] = helperEstimateRelativePose(matchedPoints1, matchedPoints2, cameraParams);
    
    % Add the current view to the view set.
    vSet = addView(vSet, i, 'Points', currPoints);
    
    % Store the point matches between the previous and the current views.
    vSet = addConnection(vSet, i-1, i, 'Matches', indexPairs(inlierIdx,:));
    
    % Get the table containing the previous camera pose.
    prevPose = poses(vSet, i-1);
    prevOrientation = prevPose.Orientation{1};
    prevLocation    = prevPose.Location{1};
        
    % Compute the current camera pose in the global coordinate system 
    % relative to the first view.
    orientation = relativeOrient * prevOrientation;
    location    = prevLocation + relativeLoc * prevOrientation;
    vSet = updateView(vSet, i, 'Orientation', orientation, ...
        'Location', location);
    
    % Find point tracks across all views.
    tracks = findTracks(vSet);

    % Get the table containing camera poses for all views.
    camPoses = poses(vSet);

    % Triangulate initial locations for the 3-D world points.
    xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
    
    % Refine the 3-D world points and camera poses.
    [xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(xyzPoints, ...
        tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);

    % Store the refined camera poses.
    vSet = updateView(vSet, camPoses);

    prevFeatures = currFeatures;
    prevPoints   = currPoints;  
end
%% Display the refined camera poses and 3-D world points.
% Display camera poses.
camPoses = poses(vSet);
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D points.
goodIdx = (reprojectionErrors < 5);
xyzPoints = xyzPoints(goodIdx, :);

% Display the 3-D points.
pcshow(xyzPoints, 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('Refined Camera Poses');
%% Compute Dense Reconstruction
% Go through the images again. This time detect a dense set of corners, and track them across all views using vision.PointTracker.
% Read and undistort the first image
I = undistortImage(images{1}, cameraParams); 

% Detect corners in the first image.
prevPoints = detectMinEigenFeatures(I, 'MinQuality', 0.001);

% Create the point tracker object to track the points across views.
tracker = vision.PointTracker('MaxBidirectionalError', 1, 'NumPyramidLevels', 6);

% Initialize the point tracker.
prevPoints = prevPoints.Location;
initialize(tracker, prevPoints, I);

% Store the dense points in the view set.
vSet = updateConnection(vSet, 1, 2, 'Matches', zeros(0, 2));
vSet = updateView(vSet, 1, 'Points', prevPoints);

% Track the points across all views.
for i = 2:numel(images)
    % Read and undistort the current image.
    I = undistortImage(images{i}, cameraParams); 
    
    % Track the points.
    [currPoints, validIdx] = step(tracker, I);
    
    % Clear the old matches between the points.
    if i < numel(images)
        vSet = updateConnection(vSet, i, i+1, 'Matches', zeros(0, 2));
    end
    vSet = updateView(vSet, i, 'Points', currPoints);
    
    % Store the point matches in the view set.
    matches = repmat((1:size(prevPoints, 1))', [1, 2]);
    matches = matches(validIdx, :);        
    vSet = updateConnection(vSet, i-1, i, 'Matches', matches);
end

% Find point tracks across all views.
tracks = findTracks(vSet);

% Find point tracks across all views.
camPoses = poses(vSet);

% Triangulate initial locations for the 3-D world points.
xyzPoints = triangulateMultiview(tracks, camPoses,...
    cameraParams);

% Refine the 3-D world points and camera poses.
[xyzPoints, camPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
    'PointsUndistorted', true);
%% Display the camera poses and the dense point cloud -Dense Reconstruction-
% Display the refined camera poses.
figure;
plotCamera(camPoses, 'Size', 0.2);
hold on

% Exclude noisy 3-D world points.
goodIdx = (reprojectionErrors < 5);

% Display the dense 3-D world points.
pcshow(xyzPoints(goodIdx, :), 'VerticalAxis', 'y', 'VerticalAxisDir', 'down', ...
    'MarkerSize', 45);
grid on
hold off

% Specify the viewing volume.
loc1 = camPoses.Location{1};
xlim([loc1(1)-5, loc1(1)+4]);
ylim([loc1(2)-5, loc1(2)+4]);
zlim([loc1(3)-1, loc1(3)+20]);
camorbit(0, -30);

title('3D-Scene Reconstruction');