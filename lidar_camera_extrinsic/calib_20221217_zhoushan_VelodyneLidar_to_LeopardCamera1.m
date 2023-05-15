clearvars; close all; clc;

addpath('../3rdpart/jsonlab');
exp_name = "20221217_zhoushan";
camera_name = "LeopardCamera1";
lidar_name = "VelodyneLidar";
squareSizeInMM = 100;
board_nrow = 10;
board_ncol = 9;
show_on = 1;
output_root = "runs";
checkboardDimensionInMM = [1200, 1200];

output_folder = fullfile(output_root, exp_name);
if ~exist(output_folder, "dir")
    mkdir(output_folder);
end

% Load instrinsic
json_data = loadjson(fullfile("data", exp_name, sprintf("%s_intrinsic.json", camera_name)));
intrinsics = cameraIntrinsics( ...
    [json_data.intrinsic_matrix(1,1), json_data.intrinsic_matrix(2,2)], ...
    [json_data.intrinsic_matrix(1,3), json_data.intrinsic_matrix(2,3)], ...
    json_data.image_size, ...
    'RadialDistortion', json_data.radial_distortion, ...
    'TangentialDistortion', json_data.tangential_distortion, ...
    'Skew', 0.000000);

% load camera frames
camera_frames = readtable(fullfile("data", exp_name, sprintf("%s.xlsx", camera_name)));

% load lidar frames
lidar_frames = dir(fullfile("data", exp_name, "VelodyneLidar"));
lidar_frames = sort_nat({lidar_frames.name});
lidar_frames = lidar_frames(3:end)';
lidar_frames = replace(lidar_frames,".pcd","");
lidar_frames = cell2table(lidar_frames, "VariableNames", "pcd_name");

% find pairs
index_used = [];
n_pairs = 0;
for i = 1:size(camera_frames, 1)
    for j = 1:size(lidar_frames, 1)
        if strcmp(camera_frames{i, 'image_name'}{1}, lidar_frames{j, 'pcd_name'}{1})
            n_pairs = n_pairs + 1;
            index_used(n_pairs, 1) = i;
            index_used(n_pairs, 2) = j;
        end
    end    
end

% detect checkboardPoints
n_points = (board_nrow - 1) * (board_ncol - 1);
imagePoints = zeros(n_points, 2, n_pairs);
for i = 1:n_pairs
    index_camera = index_used(i, 1);
    if camera_frames{index_camera, 'fix'}
        image_path = fullfile("data", exp_name, camera_name, ...
            [camera_frames{index_camera, 'image_name'}{1}, '_fix.png']);
    else
        image_path = fullfile("data", exp_name, camera_name, ...
            [camera_frames{index_camera, 'image_name'}{1}, '.png']);   
    end
    image = imread(image_path);

    if ismatrix(image)
        Igray = image;
    else
        Igray = rgb2gray(image);
    end
    Igray = im2single(Igray);
    corners_info = split(camera_frames{index_camera, 'corners'}{1}, ',');
    corners = zeros(size(corners_info, 1), 2);
    for j = 1:size(corners, 1)
        corners(j, :) = [floor(str2double(corners_info{j}) / 1e3), mod(str2double(corners_info{j}), 1e3)];
    end   
    [u, v] = meshgrid(1:size(image, 2), 1:size(image, 1));
    [in, on] = inpolygon( ...
        reshape(u, [], 1),reshape(v, [], 1), ...
        corners(:, 1), corners(:, 2));
    mask = reshape(in|on, size(image, 1), size(image, 2));
    I = Igray;
    I(~mask) = 0;
    I = adapthisteq(I);
    
    sigma = camera_frames{index_camera, 'sigma'};
    minCornerMetric = camera_frames{index_camera, 'minCornerMetric'};
    highDistortion = camera_frames{index_camera, 'highDistortion'};
    usePartial = camera_frames{index_camera, 'usePartial'};
    [points, ~] = vision.internal.calibration.checkerboard.detectCheckerboard( ...
            I, sigma, minCornerMetric, highDistortion, usePartial);
    if strcmp(camera_name, 'IRayCamera')
        points = flip(points, 1); % special for IRayCamera
    end
    imagePoints(:, :, i) = points;

    if show_on
        figure(1);

        subplot(2, 2, 1);
        imshow(image);        
        hold on;        
        text(points(:, 1), points(:, 2), split(num2str(1:size(points, 1))), ...
            'color', 'green', 'HorizontalAlignment', 'center');
        hold off; 
        title(image_path);

        subplot(2, 2, 2);
        imshow(Igray);        
        hold on;
        scatter(points(:, 1), points(:, 2), 100, 'red', '+', 'LineWidth', 2);
        hold off; 
        title('Igray');

        subplot(2, 2, 4);
        imshow(I);        
        hold on;
        scatter(points(:, 1), points(:, 2), 100, 'red', '+', 'LineWidth', 2);
        hold off; 
        title('I');

        subplot(2, 2, 3);
        imshow(image);        
        hold on;
        scatter(points(:, 1), points(:, 2), 100, 'red', '+', 'LineWidth', 2);
        hold off; 
        title(image_path);
    end

end

% create cameraParams
boardSize = [board_ncol, board_nrow];
worldPoints = generateCheckerboardPoints(boardSize, squareSizeInMM);
cameraParams = create_cameraParams(imagePoints, worldPoints, intrinsics);

% undistort imagePoints
imagePoints_undistorted = zeros(size(imagePoints));
for i = 1:size(imagePoints, 3)
    imagePoints_undistorted(:, :, i) = undistortPoints(imagePoints(:, :, i), cameraParams);
end

% get and undistort corners
n_corners = 4;
imageCorners = zeros(n_corners, 2, n_pairs);
for i = 1:n_pairs
    index_camera = index_used(i, 1);

    tmp = split(camera_frames{index_camera, 'corner1'}, ',');
    imageCorners(1, 1, i) = str2double(tmp(1));
    imageCorners(1, 2, i) = str2double(tmp(2));

    tmp = split(camera_frames{index_camera, 'corner2'}, ',');
    imageCorners(2, 1, i) = str2double(tmp(1));
    imageCorners(2, 2, i) = str2double(tmp(2));

    tmp = split(camera_frames{index_camera, 'corner3'}, ',');
    imageCorners(3, 1, i) = str2double(tmp(1));
    imageCorners(3, 2, i) = str2double(tmp(2));

    tmp = split(camera_frames{index_camera, 'corner4'}, ',');
    imageCorners(4, 1, i) = str2double(tmp(1));
    imageCorners(4, 2, i) = str2double(tmp(2));
end
imageCorners_undistorted = zeros(size(imageCorners));
for i = 1:size(imageCorners_undistorted, 3)
    imageCorners_undistorted(:, :, i) = undistortPoints(imageCorners(:, :, i), cameraParams);
end

% Compute homographies
% Compute projective transformation from worldPoints to imagePoints
[H, ~] = lidar.internal.calibration.computeHomographies(imagePoints, worldPoints);

% Compute Rotation and Translation vectors
[rvecs, tvecs] = lidar.internal.calibration.computeExtrinsics(cameraParams.IntrinsicMatrix', H);

% Extract Rotation metrices
rotationMatrices = zeros(3, 3, size(rvecs,1));
for i = 1:size(rvecs,1)
    rotationMatrices(:, :, i) = (vision.internal.calibration.rodriguesVectorToMatrix(rvecs(i, :)))';
end

% find corners from image and pointcloud
imageCorners3d = zeros(n_corners, 3, n_pairs);
lidarCheckerboardPlanes = pointCloud([0,0,0]);
lidarCorners3d = zeros(n_corners, 3, n_pairs);
% manual
index_first_manual = zeros(n_pairs, 1);
index_first_manual(14) = 2;
for i = 1:n_pairs
    % get the corners from pixel coordinate to world coordinate
    cornerWorldPts = H(:,:,i) \ [imageCorners_undistorted(:, :, i), ones(n_corners, 1)]';
    cornerWorldPts = cornerWorldPts';
    cornerWorldPts(:, 3) = 0;
    R = rotationMatrices(:, :, i)';
    t = tvecs(i, :)';
    corners = (bsxfun(@plus, R * cornerWorldPts', t))';
    corners = corners./1000;
    
    % select top point as first, clockwise
    if index_first_manual(i, :) == 0
        [~, index_first] = min(imageCorners_undistorted(:, 2, i));
    else
        index_first = index_first_manual(i, :);
    end
    index_corners = zeros(1, size(corners, 1));
    index_corners(1) = index_first;
    for j = 1 : size(corners, 1)-1
        index_next = index_first-j;
        if index_next < 1
            index_next = index_next + size(corners, 1);
        end
        index_corners(j+1) = index_next;
    end
    corners_reorder = corners(index_corners, :);
    imageCorners3d(:, :, i) = corners_reorder;

    % detect the checkboard from pointcloud
    index_lidar = index_used(i, 2);
    pcd_path = fullfile("data", exp_name, lidar_name, ...
                [lidar_frames{index_lidar, 'pcd_name'}{1}, '.pcd']);
    pcd = pcread(pcd_path);
    
    planeDimension = checkboardDimensionInMM/1000;
    tolerance = 0.4;
    minWidthToCheck = planeDimension(1) - planeDimension(1) * tolerance;
    maxWidthToCheck = planeDimension(1) + planeDimension(1) * tolerance;
    minLengthToCheck = planeDimension(2) - planeDimension(2) * tolerance;
    maxLengthToCheck = planeDimension(2) + planeDimension(2) * tolerance;    
    dimensionRange = [minWidthToCheck, maxWidthToCheck, minLengthToCheck, maxLengthToCheck];
    
    roi = [];
    minDistance = 1;
    removeGround = 0;
    [pcd_checkboard, ~, indices] = lidar.internal.calibration.extractRectangle( ...
        pcd, roi, minDistance, dimensionRange, removeGround);
    lidarCheckerboardPlanes(i) = pcd_checkboard;
    
    [corners_pcd, corners_cuboid, face_front, face_behind] = extractLidarCorners(pcd_checkboard);
    lidarCorners3d(:, :, i) = corners_pcd;

    if show_on
        index_camera = index_used(i, 1);
        if camera_frames{index_camera, 'fix'}
            image_path = fullfile("data", exp_name, camera_name, ...
                [camera_frames{index_camera, 'image_name'}{1}, '_fix.png']);
        else
            image_path = fullfile("data", exp_name, camera_name, ...
                [camera_frames{index_camera, 'image_name'}{1}, '.png']);   
        end
        image = imread(image_path);
        image_undistort = undistortImage(image, cameraParams);

        figure(2);
        
        subplot(2,2,1);
        imshow(image);
        hold on;
        text(imagePoints(:, 1, i), imagePoints(:, 2, i), split(num2str(1:n_points)), ...
            'color', 'green', 'HorizontalAlignment', 'center');
        text(imageCorners(:, 1, i), imageCorners(:, 2, i), split(num2str(1:n_corners)), ...
            'color', 'red', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        hold off;
        title(image_path, "Color", 'w');
%         figure();imshow(image);title(image_path);

        subplot(2,2,2);
        imshow(image_undistort);
        hold on;
        text(imagePoints_undistorted(:, 1, i), imagePoints_undistorted(:, 2, i), split(num2str(1:n_points)), ...
            'color', 'green', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        text(imageCorners_undistorted(index_corners, 1, i), imageCorners_undistorted(index_corners, 2, i), split(num2str(1:n_corners)), ...
            'color', 'yellow', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        hold off;
        title(sprintf("(%d/%d)image_undistort&arange_corners",i,n_pairs), "Interpreter", "none", "Color", 'w');

        subplot(2,2,3);
        pcshowpair(pcd, pcd_checkboard, 'AxesVisibility', 'on');
        title(pcd_path, 'Interpreter', 'none');
        xlabel('x');
        ylabel('y');
        zlabel('z');
        xlim([-20, 20]);
        ylim([0, 40]);    
        view([0, 90]);

        subplot(2,2,4);
        pcshowpair(pcd, pcd_checkboard, 'AxesVisibility', 'on');
        hold on;
        text(lidarCorners3d(:, 1, i), lidarCorners3d(:, 2, i), lidarCorners3d(:, 3, i), ...
            split(num2str(1:n_corners)), ...
            'color', 'cyan', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        text(imageCorners3d(:, 1, i), imageCorners3d(:, 3, i), -imageCorners3d(:, 2, i), ...
            split(num2str(1:n_corners)), ...
            'color', 'yellow', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
        hold off;
        title(pcd_path, 'Interpreter', 'none');
        xlabel('x');
        ylabel('y');
        zlabel('z');
        view([30,10]);
    end
end

% Calibrate Lidar to Camera
tform_init = computeInitialTransform(lidarCorners3d, imageCorners3d);
tform = refineTransform(lidarCorners3d, imageCorners3d, tform_init);

%% fuse visit lidar and camera
if show_on
    for i = 1:n_pairs
        index_camera = index_used(i, 1);
        if camera_frames{index_camera, 'fix'}
            image_path = fullfile("data", exp_name, camera_name, ...
                [camera_frames{index_camera, 'image_name'}{1}, '_fix.png']);
        else
            image_path = fullfile("data", exp_name, camera_name, ...
                [camera_frames{index_camera, 'image_name'}{1}, '.png']);   
        end
        image = imread(image_path);
        image_undistort = undistortImage(image, cameraParams);
        
        pcd_checkboard = lidarCheckerboardPlanes(i);

        % projection
        [uv, indice]= projectLidarPointsOnImage(pcd_checkboard, cameraParams, tform);
        
        % dye pointcloud        
        subpc = select(pcd_checkboard, indice);
        pcd_checkboard_with_color = fuseCameraToLidar(image_undistort, subpc, cameraParams, invert(tform));

        figure(3);

        subplot(2,1,1);
        imshow(image_undistort);
        hold on;
        scatter(uv(:, 1), uv(:, 2), 5, '.r');
        hold off;
        title(image_path, "Interpreter", "none", "Color", 'w');

        subplot(2,1,2);
        pcshow(pcd_checkboard_with_color, 'MarkerSize', 400, 'AxesVisibility', 'on');
        xlabel('x');
        ylabel('y');
        zlabel('z');
        title('pcd_checkboard_with_color', 'Interpreter', 'none', "Color", 'w');
    end
end

% calculate and visualize errors
[errors_translation, errors_rotation, errors_reprojection] = computeErrors( ...
    lidarCorners3d, imageCorners3d, cameraParams, tform);
helperShowError(errors_translation, errors_rotation, errors_reprojection);
savefig(fullfile(output_folder, sprintf("%s_to_%s_error.fig", lidar_name, camera_name)));

% save as json
RT = tform.A;
json_data = struct("extrinsic_matrix", RT);
json_path = fullfile(output_folder, sprintf("%s_to_%s_extrinsic.json", lidar_name, camera_name));
savejson('', json_data, json_path);