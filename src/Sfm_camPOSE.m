%% Developed by Abanob Soliman for Master Thesis in Computer Vision and Sensor Fusion 
% under the Supervision of Prof. Samia Bouchafa Head of IBISC Laboratory, University of Evry, France

function [camPoses,xyzPoints,xyzPoints1,vSet,reprojectionErrors1,reprojectionErrors2]=Sfm_camPOSE(Images,cameraPARAMS,prevloc,prevorient)
    % Structure From Motion Algorithm in order to obtain the Camera Pose and the 3D Scene Construction
    % DataSet Initialization
    images = Images;

    % Load the cameraParameters object created using the Camera Calibrator app
    if ~exist('cameraPARAMS','var')
        % third parameter does not exist, so default it to something
        data = load(fullfile(imageDir, 'cameraParams.mat'));
        cameraParams = data.cameraParams;
    else
        cameraParams = cameraPARAMS;
    end

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
    vSet = addView(vSet, viewId, 'Points', prevPoints, 'Orientation',prevorient, 'Location', prevloc);
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
        [xyzPoints1, camPoses, reprojectionErrors1] = bundleAdjustment(xyzPoints, ...
            tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
            'PointsUndistorted', true);

        % Store the refined camera poses.
        vSet = updateView(vSet, camPoses);

        prevFeatures = currFeatures;
        prevPoints   = currPoints;  
    end
    
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
    [xyzPoints, camPoses, reprojectionErrors2] = bundleAdjustment(...
        xyzPoints, tracks, camPoses, cameraParams, 'FixedViewId', 1, ...
        'PointsUndistorted', true);
   
end