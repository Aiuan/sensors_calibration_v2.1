clearvars; close all; clc;

exp_name = "20221217_zhoushan";
% exp_name = "20221221_zhoushan";
% exp_name = "20221226_yantai";

camera_name = 'IRayCamera';
% camera_name = "LeopardCamera0";
% camera_name = 'LeopardCamera1';

corners_color = ["red"; "green"; "blue"; "yellow"];
n_corners = size(corners_color, 1);

camera_frames = dir(fullfile("data", exp_name, camera_name));
camera_frames = sort_nat({camera_frames.name});
camera_frames = camera_frames(3:end)';

mask_use = false(size(camera_frames, 1), 1);
corners = zeros(size(camera_frames, 1), n_corners, 2);
for i = 1:size(camera_frames, 1)
    disp('=====================================================');

    image_path = fullfile("data", exp_name, camera_name, camera_frames{i});    
    image=imread(image_path);    
    
    figure(1);
    imshow(image);
    title(image_path);
    %使该图显示最大化，便于取点
    set(gcf,'outerposition',get(0,'screensize'));
end




