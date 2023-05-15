clear; close all; clc;

addpath('../3rdpart/jsonlab');

output_folder = './';
res_log_path = './runs/20221217_zhoushan/log.txt';
output_filename = './runs/20221217_zhoushan/MEMS_to_VelodyneLidar_extrinsic.json';

if ~exist(output_folder, 'dir')
    mkdir(output_folder);
    fprintf('create %s\n', output_folder);
end

res = readlines(res_log_path);

VelodyneLidar_to_MEMS_extrinsic = res(end-4:end-1);
for i = 1:size(VelodyneLidar_to_MEMS_extrinsic, 1)
    VelodyneLidar_to_MEMS_extrinsic(i) = strtrim(VelodyneLidar_to_MEMS_extrinsic(i));
end
VelodyneLidar_to_MEMS_extrinsic = split(VelodyneLidar_to_MEMS_extrinsic);
VelodyneLidar_to_MEMS_extrinsic = str2double(VelodyneLidar_to_MEMS_extrinsic);

MEMS_to_VelodyneLidar_extrinsic = inv(VelodyneLidar_to_MEMS_extrinsic);

RT = MEMS_to_VelodyneLidar_extrinsic;
output = struct();
output.extrinsic_matrix = RT;

output_path = fullfile(output_folder, output_filename);
savejson('', output, output_path);
fprintf('finish outputing %s\n', output_path);
