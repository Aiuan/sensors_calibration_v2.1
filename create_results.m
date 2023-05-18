clearvars; close all; clc;

addpath("./3rdpart/jsonlab");

root_output = "./results";
overwrite = true;

if overwrite
    if exist(root_output, "dir")
        rmdir(root_output, 's');
    end
end

if ~exist(root_output, "dir")
    mkdir(root_output);
end
%% zhoushan
name = "zhoushan";
output_folder = fullfile(root_output, name);
if ~exist(output_folder, "dir")
    mkdir(output_folder);
end

% IRayCamera
copyfile("./camera_intrinsic/runs/20221221_zhoushan/IRayCamera_intrinsic.json", output_folder);
copyfile("./stereo_camera_extrinsic/runs/20221221_zhoushan/IRayCamera_to_LeopardCamera0_extrinsic.json", output_folder);

% LeopardCamera0
copyfile("./camera_intrinsic/runs/20221221_zhoushan/LeopardCamera0_intrinsic.json", output_folder);

% LeopardCamera1
copyfile("./camera_intrinsic/runs/20221221_zhoushan/LeopardCamera1_intrinsic.json", output_folder);
copyfile("./stereo_camera_extrinsic/runs/20221221_zhoushan/LeopardCamera1_to_LeopardCamera0_extrinsic.json", output_folder);

% VelodyneLidar
copyfile("lidar_camera_extrinsic\runs\20221221_zhoushan\VelodyneLidar_to_LeopardCamera0_extrinsic.json", output_folder);

% MEMS
copyfile("mems_lidar_extrinsic\runs\20221217_zhoushan\MEMS_to_VelodyneLidar_extrinsic.json", output_folder);
copyfile("mems_vehicle_extrinsic\runs\20221217_zhoushan\MEMS_to_Vehicle_extrinsic.json", output_folder);

% TIRadar
copyfile("TIRadar_calib\runs\20221217_zhoushan\mode1_256x128.mat", output_folder);
copyfile("TIRadar_calib\runs\20221217_zhoushan\mode2_256x128.mat", output_folder);
copyfile("TIRadar_calib\runs\20221217_zhoushan\mode3_128x255.mat", output_folder);
copyfile("TIRadar_calib\runs\20221217_zhoushan\mode4_512x64.mat", output_folder);

copyfile("radar_camera_extrinsic\runs\zhoushan\TIRadar_to_LeopardCamera0_extrinsic.json", output_folder);
% VelodyneLidar_to_LeopardCamera0_extrinsic = loadjson( ...
%     fullfile(output_folder, "VelodyneLidar_to_LeopardCamera0_extrinsic.json"));
% TIRadar_to_LeopardCamera0_extrinsic = loadjson( ...
%     "radar_camera_extrinsic\runs\zhoushan\TIRadar_to_LeopardCamera0_extrinsic.json");
% TIRadar_to_VelodyneLidar_extrinsic = VelodyneLidar_to_LeopardCamera0_extrinsic.extrinsic_matrix \ ...
%     TIRadar_to_LeopardCamera0_extrinsic.extrinsic_matrix;
% savejson('', struct("extrinsic_matrix", TIRadar_to_VelodyneLidar_extrinsic), ...
%     fullfile(output_folder, "VelodyneLidar_to_LeopardCamera0_extrinsic.json"));

% OCULiiRadar
copyfile("radar_camera_extrinsic\runs\zhoushan\OCULiiRadar_to_LeopardCamera0_extrinsic.json", output_folder);
% VelodyneLidar_to_LeopardCamera0_extrinsic = loadjson( ...
%     fullfile(output_folder, "VelodyneLidar_to_LeopardCamera0_extrinsic.json"));
% OCULiiRadar_to_LeopardCamera0_extrinsic = loadjson( ...
%     "radar_camera_extrinsic\runs\zhoushan\OCULiiRadar_to_LeopardCamera0_extrinsic.json");
% OCULiiRadar_to_VelodyneLidar_extrinsic = VelodyneLidar_to_LeopardCamera0_extrinsic.extrinsic_matrix \ ...
%     OCULiiRadar_to_LeopardCamera0_extrinsic.extrinsic_matrix;
% savejson('', struct("extrinsic_matrix", OCULiiRadar_to_VelodyneLidar_extrinsic), ...
%     fullfile(output_folder, "OCULiiRadar_to_VelodyneLidar_extrinsic.json"));

%% yantai
name = "yantai";
output_folder = fullfile(root_output, name);
if ~exist(output_folder, "dir")
    mkdir(output_folder);
end

% IRayCamera
copyfile("./camera_intrinsic/runs/20221226_yantai/IRayCamera_intrinsic.json", output_folder);
copyfile("./stereo_camera_extrinsic/runs/20221226_yantai/IRayCamera_to_LeopardCamera0_extrinsic.json", output_folder);

% LeopardCamera0
copyfile("./camera_intrinsic/runs/20221226_yantai/LeopardCamera0_intrinsic.json", output_folder);

% LeopardCamera1
copyfile("./camera_intrinsic/runs/20221226_yantai/LeopardCamera1_intrinsic.json", output_folder);
copyfile("./stereo_camera_extrinsic/runs/20221226_yantai/LeopardCamera1_to_LeopardCamera0_extrinsic.json", output_folder);

% VelodyneLidar
VelodyneLidar_to_IRayCamera_extrinsic = loadjson("lidar_camera_extrinsic\runs\20221226_yantai\VelodyneLidar_to_IRayCamera_extrinsic.json");
IRayCamera_to_LeopardCamera0_extrinsic = loadjson("./stereo_camera_extrinsic/runs/20221226_yantai/IRayCamera_to_LeopardCamera0_extrinsic.json");
VelodyneLidar_to_LeopardCamera0_extrinsic = IRayCamera_to_LeopardCamera0_extrinsic.extrinsic_matrix * ...
    VelodyneLidar_to_IRayCamera_extrinsic.extrinsic_matrix;
savejson('', struct("extrinsic_matrix", VelodyneLidar_to_LeopardCamera0_extrinsic), ...
    fullfile(output_folder, "VelodyneLidar_to_LeopardCamera0_extrinsic.json"));

% MEMS
copyfile("mems_lidar_extrinsic\runs\20221226_yantai\MEMS_to_VelodyneLidar_extrinsic.json", output_folder);
copyfile("mems_vehicle_extrinsic\runs\20221226_yantai\MEMS_to_Vehicle_extrinsic.json", output_folder);

% TIRadar
copyfile("TIRadar_calib\runs\20221226_yantai\mode1_256x128.mat", output_folder);
copyfile("TIRadar_calib\runs\20221226_yantai\mode2_256x128.mat", output_folder);
copyfile("TIRadar_calib\runs\20221226_yantai\mode3_128x255.mat", output_folder);
copyfile("TIRadar_calib\runs\20221226_yantai\mode4_512x64.mat", output_folder);

copyfile("radar_camera_extrinsic\runs\yantai\TIRadar_to_LeopardCamera0_extrinsic.json", output_folder);
% VelodyneLidar_to_LeopardCamera0_extrinsic = loadjson( ...
%     fullfile(output_folder, "VelodyneLidar_to_LeopardCamera0_extrinsic.json"));
% TIRadar_to_LeopardCamera0_extrinsic = loadjson( ...
%     "radar_camera_extrinsic\runs\yantai\TIRadar_to_LeopardCamera0_extrinsic.json");
% TIRadar_to_VelodyneLidar_extrinsic = VelodyneLidar_to_LeopardCamera0_extrinsic.extrinsic_matrix \ ...
%     TIRadar_to_LeopardCamera0_extrinsic.extrinsic_matrix;
% savejson('', struct("extrinsic_matrix", TIRadar_to_VelodyneLidar_extrinsic), ...
%     fullfile(output_folder, "VelodyneLidar_to_LeopardCamera0_extrinsic.json"));

% OCULiiRadar
copyfile("radar_camera_extrinsic\runs\yantai\OCULiiRadar_to_LeopardCamera0_extrinsic.json", output_folder);
% VelodyneLidar_to_LeopardCamera0_extrinsic = loadjson( ...
%     fullfile(output_folder, "VelodyneLidar_to_LeopardCamera0_extrinsic.json"));
% OCULiiRadar_to_LeopardCamera0_extrinsic = loadjson( ...
%     "radar_camera_extrinsic\runs\yantai\OCULiiRadar_to_LeopardCamera0_extrinsic.json");
% OCULiiRadar_to_VelodyneLidar_extrinsic = VelodyneLidar_to_LeopardCamera0_extrinsic.extrinsic_matrix \ ...
%     OCULiiRadar_to_LeopardCamera0_extrinsic.extrinsic_matrix;
% savejson('', struct("extrinsic_matrix", OCULiiRadar_to_VelodyneLidar_extrinsic), ...
%     fullfile(output_folder, "OCULiiRadar_to_VelodyneLidar_extrinsic.json"));
