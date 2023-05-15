clear; close all; clc;

addpath('../3rdpart/jsonlab');

output_folder = './runs/20221226_yantai';
res_csv_path = './runs/20221226_yantai/imu_heading.csv';
output_filename = 'MEMS_to_Vehicle_extrinsic.json';

if ~exist(output_folder, 'dir')
    mkdir(output_folder);
    fprintf('create %s\n', output_folder);
end

res = readlines(res_csv_path);
res = res(1:end-1, :);
res = split(res, ',');

% ENU坐标系定义的旋转角是顺时针为正
MEMS_to_Vehicle_yaw_degree = -str2double(res(2,3));
% 手动测量
MEMS_to_Vehicle_x_m = -0.2;
MEMS_to_Vehicle_y_m = 0;
MEMS_to_Vehicle_z_m = 0.8;

% 坐标系转动
% 顺序ZXY，内旋角
R = [
    cosd(MEMS_to_Vehicle_yaw_degree), sind(MEMS_to_Vehicle_yaw_degree), 0;
    -sind(MEMS_to_Vehicle_yaw_degree), cosd(MEMS_to_Vehicle_yaw_degree), 0;
    0, 0, 1
    ];
RT = zeros(4, 4);
RT(1:3, 1:3) = R;
RT(1, 4) = MEMS_to_Vehicle_x_m;
RT(2, 4) = MEMS_to_Vehicle_y_m;
RT(3, 4) = MEMS_to_Vehicle_z_m;
RT(4, 4) = 1;

output = struct();
output.extrinsic_matrix = RT;

output_path = fullfile(output_folder, output_filename);
savejson('', output, output_path);
fprintf('finish outputing %s\n', output_path);

%% vis
clearvars; close all; clc;

file_path = './runs/20221226_yantai/heading_imu_input.csv';
data = readlines(file_path);
data = data(1:end-1, :);
data = split(data, ',');

x = str2double(data(2:end, 2));
y = str2double(data(2:end, 3));
z = str2double(data(2:end, 4));
idx = (1:size(data)-1)';

figure(1);
scatter3(x,y,z,1,idx,'filled');
grid on;
colormap('jet');
xlabel('x');
ylabel('y');
zlabel('z');
colorbar();

% axis('equal');