clearvars; close all; clc;
addpath("../3rdpart/jsonlab");

name = 'yantai';
root_data = fullfile("./data", name);
root_calib = fullfile("../results", name);

labels = loadjson(fullfile(root_data, "label.json"));
image_IRayCamera = imread(fullfile(root_data, "IRayCamera.png"));
image_LeopardCamera0 = imread(fullfile(root_data, "LeopardCamera0.png"));
image_LeopardCamera1 = imread(fullfile(root_data, "LeopardCamera1.png"));
pcd_VelodyneLidar = load_VelodyneLidar_pcd(fullfile(root_data, "VelodyneLidar.pcd"));
pcd_TIRadar = load_TIRadar_pcd(fullfile(root_data, "TIRadar.pcd"));
pcd_OCULiiRadar = load_OCULiiRadar_pcd(fullfile(root_data, "OCULiiRadar.pcd"));

IRayCamera_intrinsic = load_intrinsic(fullfile(root_calib, "IRayCamera_intrinsic.json"));
IRayCamera_to_LeopardCamera0_extrinsic = load_extrinsic( ...
    fullfile(root_calib, "IRayCamera_to_LeopardCamera0_extrinsic.json"));
LeopardCamera0_intrinsic = load_intrinsic(fullfile(root_calib, "LeopardCamera0_intrinsic.json"));
LeopardCamera1_intrinsic = load_intrinsic(fullfile(root_calib, "LeopardCamera1_intrinsic.json"));
LeopardCamera1_to_LeopardCamera0_extrinsic = load_extrinsic( ...
    fullfile(root_calib, "LeopardCamera1_to_LeopardCamera0_extrinsic.json"));
VelodyneLidar_to_LeopardCamera0_extrinsic = load_extrinsic( ...
    fullfile(root_calib, "VelodyneLidar_to_LeopardCamera0_extrinsic.json"));

%% VelodyneLidar_to_LeopardCamera0_extrinsic
figure();
image_LeopardCamera0_unditorted = undistortImage(image_LeopardCamera0, LeopardCamera0_intrinsic);
imshow(image_LeopardCamera0);
title("LeopardCamera0");

for i = 1:length(labels)
    x = labels(i).x;
    y = labels(i).y;
    z = labels(i).z;
    l = labels(i).l;
    w = labels(i).w;
    h = labels(i).h;
    alpha = labels(i).alpha;
    class = labels(i).class;
    object_id = labels(i).object_id;

    corners = [
        l/2,  l/2,  l/2,  l/2, -l/2, -l/2, -l/2, -l/2;
        w/2,  w/2, -w/2, -w/2,  w/2,  w/2, -w/2, -w/2;
        h/2, -h/2, -h/2,  h/2,  h/2, -h/2, -h/2,  h/2];

    rot = [
         cos(alpha), -sin(alpha), 0;
         sin(alpha),  cos(alpha), 0;
                  0,          0, 1];
    
    bbox = rot * corners + [x; y; z];
    [uv, ~]= projectLidarPointsOnImage(pointCloud(bbox'), ...
        LeopardCamera0_intrinsic, VelodyneLidar_to_LeopardCamera0_extrinsic);
    uv = uv';
    if size(uv, 2) == size(bbox, 2)
        line(uv(1, [1:4, 1]), uv(2, [1:4, 1]), "Color", "r", "LineWidth", 2);
        line(uv(1, [5:8, 5]), uv(2, [5:8, 5]), "Color", "b");
        line(uv(1, [1, 5]), uv(2, [1, 5]), "Color", "b");
        line(uv(1, [2, 6]), uv(2, [2, 6]), "Color", "b");
        line(uv(1, [3, 7]), uv(2, [3, 7]), "Color", "b");
        line(uv(1, [4, 8]), uv(2, [4, 8]), "Color", "b");
        text(mean(uv(1, [1,4])), mean(uv(2, [1,4])), sprintf("%s %s", object_id, class), ...
            "HorizontalAlignment", "center", "VerticalAlignment", "bottom", "Color", "b");
    end    
end
