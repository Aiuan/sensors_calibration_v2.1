%% find corner reflector in pixel coordinate
clearvars; close all; clc;

% exp_name = "zhoushan";
exp_name = "yantai";

camera_name = 'IRayCamera';
% camera_name = "LeopardCamera0";
% camera_name = 'LeopardCamera1';

save_on = 1;
output_path = fullfile("./data", exp_name, sprintf("%s.xlsx", camera_name));

image_folder= fullfile("./data", exp_name, camera_name);
image_list = dir(image_folder);
image_list_name = sort_nat({image_list.name});
image_list_name = image_list_name(3:end)';

points_pixel = zeros(length(image_list_name), 2);
figure(1);
for i=1:length(image_list_name)
    disp('=====================================================');
    image_path = fullfile(image_folder, image_list_name{i});
    
    image=imread(image_path);
    imshow(image);
    title(image_path, 'Interpreter', 'none');
    set(gcf,'outerposition',get(0,'screensize'));%使该图显示最大化，便于取点
    
    prompt = '是否使用此帧? Y/N: ';
    str = input(prompt, 's');
    if strcmp(str, 'Y') || strcmp(str, 'y')
        disp('请选取目标点');
        [points_pixel(i,1), points_pixel(i,2)] = ginput(1);
        hold on;
        scatter(points_pixel(i,1), points_pixel(i,2), 'yellow', 'filled');
        hold off;
        pause(1);
    else
        continue;
    end

end

% save results
if save_on
    writematrix(points_pixel, output_path);
end
