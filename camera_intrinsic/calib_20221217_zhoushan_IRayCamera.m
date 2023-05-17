clearvars; close all; clc;

exp_name = "20221217_zhoushan";
camera_name = "IRayCamera";
squareSizeInMM = 100;
board_nrow = 10;
board_ncol = 9;
show_on = 1;
output_root = "runs";
error_mean_thred = 0.2;
error_std_thred = 0.1;
n_thred = 10;

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

%         savefig(fullfile(output_folder, ...
%             sprintf("%s_%d_%s.fig", camera_name, i, params{i, 'image_name'}{1})));
    end
    
    fprintf("%s detect %d points\n", image_path, size(points, 1));

end

imagesUsed = true(size(params, 1), 1);
worldPoints = generateCheckerboardPoints(boardSize, squareSizeInMM);
imageSize = [size(image, 1), size(image, 2)];
% Calibrate the camera
while true
    if exist("errors", "var")
        [~, i_error] = max(errors);
        i_use = find(imagesUsed);
        imagesUsed(i_use(i_error)) = false;
    end

    cameraParams = estimateCameraParameters( ...
        imagePoints(:, :, imagesUsed), worldPoints(:, :), ...
        'EstimateSkew', false, 'EstimateTangentialDistortion', true, ...
        'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
        'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], 'ImageSize', imageSize);
    errors = mean(sqrt(squeeze(cameraParams.ReprojectionErrors(:, 1, :)).^2 + ...
        squeeze(cameraParams.ReprojectionErrors(:, 1, :)).^2), 1);

    figure(2);
    bar(errors);
    xticks(1:size(errors, 2));
    xticklabels(replace(params{imagesUsed, 'image_name'}, "_", "\_"));
    yline(mean(errors), '--r');
    text(size(errors, 2), mean(errors), sprintf("mean=%.2f\nstd=%.2f", mean(errors), std(errors)), ...
        "HorizontalAlignment", "center", "VerticalAlignment", "bottom", "Color", "red");
    ylabel("ReprojectionErrors");
    grid on;

    if mean(errors) <= error_mean_thred
        break;
    end

    if std(errors) <= error_std_thred
        break;
    end

    if sum(imagesUsed) <= n_thred
        break;
    end
end
savefig(fullfile(output_folder, sprintf("%s_error.fig", camera_name)));

% Visualize pattern locations
figure();
showExtrinsics(cameraParams, 'CameraCentric');

% save as json
addpath('../3rdpart/jsonlab');
json_data = struct("image_size", cameraParams.ImageSize, ...
    "intrinsic_matrix", cameraParams.K, ...
    "radial_distortion", cameraParams.RadialDistortion, ...
    "tangential_distortion", cameraParams.TangentialDistortion);
json_path = fullfile(output_folder, sprintf("%s_intrinsic.json", camera_name));
savejson('', json_data, json_path);

% save cameraParams
% save(fullfile(output_folder, sprintf("%s.mat", camera_name)), "cameraParams");
