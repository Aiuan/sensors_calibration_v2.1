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
TIRadar_to_LeopardCamera0_extrinsic = load_extrinsic( ...
    fullfile(root_calib, "TIRadar_to_LeopardCamera0_extrinsic.json"));
OCULiiRadar_to_LeopardCamera0_extrinsic = load_extrinsic( ...
    fullfile(root_calib, "OCULiiRadar_to_LeopardCamera0_extrinsic.json"));



%% lidar with bbox
figure();
pcshow(pcd_VelodyneLidar, "AxesVisibility", "on");
axis("equal");
grid on;
colormap("jet");
c = colorbar;
c.Label.String = 'intensity';
c.Color = 'w';
c.Limits = [0, 255];
xlabel("x");
ylabel("y");
zlabel("z");
title("VelodyneLidar");
view([-10, 40]);

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

    line(bbox(1, [1:4, 1]), bbox(2, [1:4, 1]), bbox(3, [1:4, 1]), "Color", "r", "LineWidth", 2);
    line(bbox(1, [5:8, 5]), bbox(2, [5:8, 5]), bbox(3, [5:8, 5]), "Color", "w");
    line(bbox(1, [1, 5]), bbox(2, [1, 5]), bbox(3, [1, 5]), "Color", "w");
    line(bbox(1, [2, 6]), bbox(2, [2, 6]), bbox(3, [2, 6]), "Color", "w");
    line(bbox(1, [3, 7]), bbox(2, [3, 7]), bbox(3, [3, 7]), "Color", "w");
    line(bbox(1, [4, 8]), bbox(2, [4, 8]), bbox(3, [4, 8]), "Color", "w");
    text( ...
        mean(bbox(1, [1,4])), mean(bbox(2, [1,4])), mean(bbox(3, [1,4])), ...
        sprintf("%s %s", object_id, class), ...
        "HorizontalAlignment", "center", "VerticalAlignment", "bottom", "Color", "w");
end

%% OCULiiRadar with bbox
[tmp_x, tmp_y, tmp_z] = transformPointsForward(OCULiiRadar_to_LeopardCamera0_extrinsic, ...
    pcd_OCULiiRadar.Location(:, 1), pcd_OCULiiRadar.Location(:, 2), pcd_OCULiiRadar.Location(:, 3));
[tmp_x, tmp_y, tmp_z] = transformPointsInverse(VelodyneLidar_to_LeopardCamera0_extrinsic, tmp_x, tmp_y, tmp_z);
pcd_OCULiiRadar_lidar_coordinate = pointCloud([tmp_x, tmp_y, tmp_z], "Intensity", pcd_OCULiiRadar.Intensity);

figure();
pcshow(pcd_OCULiiRadar_lidar_coordinate, "AxesVisibility", "on", "MarkerSize", 100);
axis("equal");
grid on;
colormap("jet");
c = colorbar;
c.Label.String = 'doppler';
c.Color = 'w';
xlabel("x");
ylabel("y");
zlabel("z");
title("OCULiiRadar");
view([-10, 40]);

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

    line(bbox(1, [1:4, 1]), bbox(2, [1:4, 1]), bbox(3, [1:4, 1]), "Color", "r", "LineWidth", 2);
    line(bbox(1, [5:8, 5]), bbox(2, [5:8, 5]), bbox(3, [5:8, 5]), "Color", "w");
    line(bbox(1, [1, 5]), bbox(2, [1, 5]), bbox(3, [1, 5]), "Color", "w");
    line(bbox(1, [2, 6]), bbox(2, [2, 6]), bbox(3, [2, 6]), "Color", "w");
    line(bbox(1, [3, 7]), bbox(2, [3, 7]), bbox(3, [3, 7]), "Color", "w");
    line(bbox(1, [4, 8]), bbox(2, [4, 8]), bbox(3, [4, 8]), "Color", "w");
    text( ...
        mean(bbox(1, [1,4])), mean(bbox(2, [1,4])), mean(bbox(3, [1,4])), ...
        sprintf("%s %s", object_id, class), ...
        "HorizontalAlignment", "center", "VerticalAlignment", "bottom", "Color", "w");
end

%% TIRadar with bbox
[tmp_x, tmp_y, tmp_z] = transformPointsForward(TIRadar_to_LeopardCamera0_extrinsic, ...
    pcd_TIRadar.Location(:, 1), pcd_TIRadar.Location(:, 2), pcd_TIRadar.Location(:, 3));
[tmp_x, tmp_y, tmp_z] = transformPointsInverse(VelodyneLidar_to_LeopardCamera0_extrinsic, tmp_x, tmp_y, tmp_z);
pcd_TIRadar_lidar_coordinate = pointCloud([tmp_x, tmp_y, tmp_z], "Intensity", pcd_TIRadar.Intensity);

figure();
pcshow(pcd_TIRadar_lidar_coordinate, "AxesVisibility", "on", "MarkerSize", 100);
axis("equal");
grid on;
colormap("jet");
c = colorbar;
c.Label.String = 'doppler';
c.Color = 'w';
xlabel("x");
ylabel("y");
zlabel("z");
title("TIRadar");
view([-10, 40]);

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

    line(bbox(1, [1:4, 1]), bbox(2, [1:4, 1]), bbox(3, [1:4, 1]), "Color", "r", "LineWidth", 2);
    line(bbox(1, [5:8, 5]), bbox(2, [5:8, 5]), bbox(3, [5:8, 5]), "Color", "w");
    line(bbox(1, [1, 5]), bbox(2, [1, 5]), bbox(3, [1, 5]), "Color", "w");
    line(bbox(1, [2, 6]), bbox(2, [2, 6]), bbox(3, [2, 6]), "Color", "w");
    line(bbox(1, [3, 7]), bbox(2, [3, 7]), bbox(3, [3, 7]), "Color", "w");
    line(bbox(1, [4, 8]), bbox(2, [4, 8]), bbox(3, [4, 8]), "Color", "w");
    text( ...
        mean(bbox(1, [1,4])), mean(bbox(2, [1,4])), mean(bbox(3, [1,4])), ...
        sprintf("%s %s", object_id, class), ...
        "HorizontalAlignment", "center", "VerticalAlignment", "bottom", "Color", "w");
end

%% IRayCamera with bbox
figure();
image_IRayCamera_unditorted = undistortImage(image_IRayCamera, IRayCamera_intrinsic);
imshow(image_IRayCamera);
title("IRayCamera");

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

    VelodyneLidar_to_IRayCamera_extrinsic = rigidtform3d(invert(IRayCamera_to_LeopardCamera0_extrinsic).A * ...
        VelodyneLidar_to_LeopardCamera0_extrinsic.A);
    [uv, ~]= projectLidarPointsOnImage(pointCloud(bbox'), ...
        IRayCamera_intrinsic, VelodyneLidar_to_IRayCamera_extrinsic);
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

%% LeopardCamera0 with bbox
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

%% LeopardCamera1 with bbox
figure();
image_LeopardCamera1_unditorted = undistortImage(image_LeopardCamera1, LeopardCamera1_intrinsic);
imshow(image_LeopardCamera1);
title("LeopardCamera1");

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

    VelodyneLidar_to_LeopardCamera1_extrinsic = rigidtform3d(invert(LeopardCamera1_to_LeopardCamera0_extrinsic).A * ...
        VelodyneLidar_to_LeopardCamera0_extrinsic.A);
    [uv, ~]= projectLidarPointsOnImage(pointCloud(bbox'), ...
        LeopardCamera1_intrinsic, VelodyneLidar_to_LeopardCamera1_extrinsic);
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

%% LeopardCamera0 & LeopardCamera1
stereoParams = stereoParameters(LeopardCamera0_intrinsic, LeopardCamera1_intrinsic, invert(LeopardCamera1_to_LeopardCamera0_extrinsic));
[I0Rect,I1Rect] = rectifyStereoImages(image_LeopardCamera0, image_LeopardCamera0, stereoParams);
figure(); 
imshow(stereoAnaglyph(I0Rect,I1Rect));
