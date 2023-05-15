clearvars; close all; clc;

addpath('../3rdpart/jsonlab');
exp_name = "20221226_yantai";
camera_name1 = "LeopardCamera0";
camera_name2 = "LeopardCamera1";
squareSizeInMM = 98;
board_nrow = 9;
board_ncol = 7;
show_on = 1;
output_root = "runs";

output_folder = fullfile(output_root, exp_name);
if ~exist(output_folder, "dir")
    mkdir(output_folder);
end

% Load instrinsic
json1 = loadjson(fullfile("data", exp_name, sprintf("%s_intrinsic.json", camera_name1)));
intrinsics1 = cameraIntrinsics( ...
    [json1.intrinsic_matrix(1,1), json1.intrinsic_matrix(2,2)], ...
    [json1.intrinsic_matrix(1,3), json1.intrinsic_matrix(2,3)], ...
    json1.image_size, ...
    'RadialDistortion', json1.radial_distortion, ...
    'TangentialDistortion', json1.tangential_distortion, ...
    'Skew', 0.000000);

json2 = loadjson(fullfile("data", exp_name, sprintf("%s_intrinsic.json", camera_name2)));
intrinsics2 = cameraIntrinsics( ...
    [json2.intrinsic_matrix(1,1), json2.intrinsic_matrix(2,2)], ...
    [json2.intrinsic_matrix(1,3), json2.intrinsic_matrix(2,3)], ...
    json2.image_size, ...
    'RadialDistortion', json2.radial_distortion, ...
    'TangentialDistortion', json2.tangential_distortion, ...
    'Skew', 0.000000);

% read parameters
params1 = readtable(fullfile("data", exp_name, sprintf("%s.xlsx", camera_name1)));
params2 = readtable(fullfile("data", exp_name, sprintf("%s.xlsx", camera_name2)));
index_used = [];
n_pairs = 0;
for i = 1:size(params1, 1)
    for j = 1:size(params2, 1)
        if strcmp(params1{i, 'image_name'}{1}, params2{j, 'image_name'}{1})
            n_pairs = n_pairs + 1;
            index_used(n_pairs, 1) = i;
            index_used(n_pairs, 2) = j;
        end
    end    
end

n_points = (board_nrow - 1) * (board_ncol - 1);
imagePoints = zeros(n_points, 2, n_pairs, 2);
for i = 1:n_pairs
    % camera1
    index_used1 = index_used(i, 1);
    if params1{index_used1, 'fix'}
        image_path1 = fullfile("data", exp_name, camera_name1, ...
            [params1{index_used1, 'image_name'}{1}, '_fix.png']);
    else
        image_path1 = fullfile("data", exp_name, camera_name1, ...
            [params1{index_used1, 'image_name'}{1}, '.png']);   
    end
    image1 = imread(image_path1);
    
    if ismatrix(image1)
        Igray1 = image1;
    else
        Igray1 = rgb2gray(image1);
    end
    Igray1 = im2single(Igray1);

    corners_info1 = split(params1{index_used1, 'corners'}{1}, ',');
    corners1 = zeros(size(corners_info1, 1), 2);
    for j = 1:size(corners1, 1)
        corners1(j, :) = [floor(str2double(corners_info1{j}) / 1e3), ...
            mod(str2double(corners_info1{j}), 1e3)];
    end
    [u1, v1] = meshgrid(1:size(image1, 2), 1:size(image1, 1));
    [in1, on1] = inpolygon( ...
        reshape(u1, [], 1),reshape(v1, [], 1), ...
        corners1(:, 1), corners1(:, 2));
    mask1 = reshape(in1|on1, size(image1, 1), size(image1, 2));    
    I1 = Igray1;
    I1(~mask1) = 0;
    I1 = adapthisteq(I1);
    
    sigma1 = params1{index_used1, 'sigma'};
    minCornerMetric1 = params1{index_used1, 'minCornerMetric'};
    highDistortion1 = params1{index_used1, 'highDistortion'};
    usePartial1 = params1{index_used1, 'usePartial'};
    [points1, boardSize1] = vision.internal.calibration.checkerboard.detectCheckerboard( ...
            I1, sigma1, minCornerMetric1, highDistortion1, usePartial1);
    imagePoints(:, :, i, 1) = points1;

    
    % camera2
    index_used2 = index_used(i, 2);
    if params2{index_used2, 'fix'}
        image_path2 = fullfile("data", exp_name, camera_name2, ...
            [params2{index_used2, 'image_name'}{1}, '_fix.png']);
    else
        image_path2 = fullfile("data", exp_name, camera_name2, ...
            [params2{index_used2, 'image_name'}{1}, '.png']);   
    end
    image2 = imread(image_path2);
    
    if ismatrix(image2)
        Igray2 = image2;
    else
        Igray2 = rgb2gray(image2);
    end
    Igray2 = im2single(Igray2);

    corners_info2 = split(params2{index_used2, 'corners'}{1}, ',');
    corners2 = zeros(size(corners_info2, 1), 2);
    for j = 1:size(corners2, 1)
        corners2(j, :) = [floor(str2double(corners_info2{j}) / 1e3), ...
            mod(str2double(corners_info2{j}), 1e3)];
    end
    [u2, v2] = meshgrid(1:size(image2, 2), 1:size(image2, 1));
    [in2, on2] = inpolygon( ...
        reshape(u2, [], 1),reshape(v2, [], 1), ...
        corners2(:, 1), corners2(:, 2));
    mask2 = reshape(in2|on2, size(image2, 1), size(image2, 2));    
    I2 = Igray2;
    I2(~mask2) = 0;
    I2 = adapthisteq(I2);
    
    sigma2 = params2{index_used2, 'sigma'};
    minCornerMetric2 = params2{index_used2, 'minCornerMetric'};
    highDistortion2 = params2{index_used2, 'highDistortion'};
    usePartial2 = params2{index_used2, 'usePartial'};
    [points2, boardSize2] = vision.internal.calibration.checkerboard.detectCheckerboard( ...
            I2, sigma2, minCornerMetric2, highDistortion2, usePartial2);
    if strcmp(camera_name2, 'IRayCamera')
        points2 = flip(points2, 1); % special for IRayCamera
    end
    imagePoints(:, :, i, 2) = points2;

    if show_on
        figure(1);

        subplot(2, 2, 1);
        imshow(image1);        
        hold on;        
        text(points1(:, 1), points1(:, 2), split(num2str(1:size(points1, 1))), ...
            'color', 'green', 'HorizontalAlignment', 'center');
        hold off; 
        title(image_path1);

        subplot(2, 2, 2);
        imshow(I1);        
        hold on;
        scatter(points1(:, 1), points1(:, 2), 100, 'red', '+', 'LineWidth', 2);
        hold off; 
        title('I1');

        subplot(2, 2, 3);
        imshow(image2);        
        hold on;        
        text(points2(:, 1), points2(:, 2), split(num2str(1:size(points2, 1))), ...
            'color', 'green', 'HorizontalAlignment', 'center');
        hold off; 
        title(image_path2);

        subplot(2, 2, 4);
        imshow(I2);        
        hold on;
        scatter(points2(:, 1), points2(:, 2), 100, 'red', '+', 'LineWidth', 2);
        hold off; 
        title('I2');
    end

end

% Calibrate stereo camera
boardSize = [board_ncol, board_nrow];
worldPoints = generateCheckerboardPoints(boardSize, squareSizeInMM);
cameraParams1 = create_cameraParams(imagePoints(:,:,:,1), worldPoints, intrinsics1);
cameraParams2 = create_cameraParams(imagePoints(:,:,:,2), worldPoints, intrinsics2);
[R, t] = vision.internal.calibration.estimateInitialTranslationAndRotation(...
    cameraParams1, cameraParams2);
stereoParams = stereoParameters(cameraParams1, cameraParams2, R, t);

% Calibrate reprojection errors
isIntrinsicsFixed = true;
shouldComputeErrors = 1;
estimationErrors = refine(stereoParams, imagePoints(:,:,:,1), ...
    imagePoints(:,:,:,2), shouldComputeErrors, isIntrinsicsFixed);

% View reprojection errors
figure();
showReprojectionErrors(stereoParams);
savefig(fullfile(output_folder, sprintf("%s_to_%s_error.fig", camera_name2, camera_name1)));

% Visualize pattern locations
figure();
showExtrinsics(stereoParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, stereoParams);

% save as json
RT = stereoParams.PoseCamera2.A;
RT(1:3, 4) = RT(1:3, 4) / 1e3;
RT = inv(RT);
json_data = struct("extrinsic_matrix", RT);
json_path = fullfile(output_folder, sprintf("%s_to_%s_extrinsic.json", camera_name2, camera_name1));
savejson('', json_data, json_path);

% save stereoParams
save(fullfile(output_folder, sprintf("%s_to_%s.mat", camera_name2, camera_name1)), "stereoParams");
