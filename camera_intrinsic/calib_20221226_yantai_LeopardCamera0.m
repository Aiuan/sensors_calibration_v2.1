clearvars; close all; clc;

exp_name = "20221226_yantai";
camera_name = "LeopardCamera0";
squareSizeInMM = 98;
board_nrow = 9;
board_ncol = 7;
show_on = 1;
output_root = "runs";

output_folder = fullfile(output_root, exp_name);
if ~exist(output_folder, "dir")
    mkdir(output_folder);
end

% read parameters
params = readtable(fullfile("data", exp_name, sprintf("%s.xlsx", camera_name)));

n_points = (board_nrow - 1) * (board_ncol - 1);
imagePoints = zeros(n_points, 2, size(params, 1));
for i = 1:size(params, 1)    
    if params{i, 'fix'}
        image_path = fullfile("data", exp_name, camera_name, [params{i, 'image_name'}{1}, '_fix.png']);
    else
        image_path = fullfile("data", exp_name, camera_name, [params{i, 'image_name'}{1}, '.png']);   
    end
    image = imread(image_path);       
%     figure(1);imshow(image);title(image_path);
%     continue;

    if ismatrix(image)
        Igray = image;
    else
        Igray = rgb2gray(image);
    end
    Igray = im2single(Igray);
%     figure();imshow(Igray);

%     I = adapthisteq(Igray);
%     figure();imshow(I);
    corners_info = split(params{i, 'corners'}{1}, ',');
    corners = zeros(size(corners_info, 1), 2);
    for j = 1:size(corners, 1)
        corners(j, :) = [floor(str2num(corners_info{j}) / 1e3), mod(str2num(corners_info{j}), 1e3)];
    end   
    [u, v] = meshgrid(1:size(image, 2), 1:size(image, 1));
    [in, on] = inpolygon( ...
        reshape(u, [], 1),reshape(v, [], 1), ...
        corners(:, 1), corners(:, 2));
    mask = reshape(in|on, size(image, 1), size(image, 2));
%     figure();imshow(mask);    
    I = Igray;
    I(~mask) = 0;
    I = adapthisteq(I);
%     figure();imshow(I);title(image_path);
%     continue;
    
    sigma = params{i, 'sigma'};
    minCornerMetric = params{i, 'minCornerMetric'};
    highDistortion = params{i, 'highDistortion'};
    usePartial = params{i, 'usePartial'};
    [points, boardSize] = vision.internal.calibration.checkerboard.detectCheckerboard( ...
            I, sigma, minCornerMetric, highDistortion, usePartial);
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
    
    fprintf("%s detect %d points\n", image_path, size(points, 1));

end

worldPoints = generateCheckerboardPoints(boardSize, squareSizeInMM);
imageSize = [size(image, 1), size(image, 2)];
% Calibrate the camera
[cameraParams, imagesUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', false, 'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], 'ImageSize', imageSize);

% View reprojection errors
figure();
showReprojectionErrors(cameraParams);
savefig(fullfile(output_folder, sprintf("%s_error.fig", camera_name)));

% Visualize pattern locations
figure();
showExtrinsics(cameraParams, 'CameraCentric');

% Display parameter estimation errors
displayErrors(estimationErrors, cameraParams);

% save as json
addpath('../3rdpart/jsonlab');
json_data = struct("image_size", cameraParams.ImageSize, ...
    "intrinsic_matrix", cameraParams.K, ...
    "radial_distortion", cameraParams.RadialDistortion, ...
    "tangential_distortion", cameraParams.TangentialDistortion);
json_path = fullfile(output_folder, sprintf("%s_intrinsic.json", camera_name));
savejson('', json_data, json_path);

% save cameraParams
save(fullfile(output_folder, sprintf("%s.mat", camera_name)), "cameraParams");
