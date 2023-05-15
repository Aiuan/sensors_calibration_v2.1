clearvars; close all; clc;

addpath('../3rdpart/jsonlab');
exp_name = "zhoushan";
camera_name = "IRayCamera";
radar_name = "OCULiiRadar";
show_on = 1;
output_root = "runs";

output_folder = fullfile(output_root, exp_name);
if ~exist(output_folder, "dir")
    mkdir(output_folder);
end

% Load instrinsic
json_data = loadjson(fullfile("data", exp_name, sprintf("%s_intrinsic.json", camera_name)));
intrinsics = cameraIntrinsics( ...
    [json_data.intrinsic_matrix(1,1), json_data.intrinsic_matrix(2,2)], ...
    [json_data.intrinsic_matrix(1,3), json_data.intrinsic_matrix(2,3)], ...
    json_data.image_size, ...
    'RadialDistortion', json_data.radial_distortion, ...
    'TangentialDistortion', json_data.tangential_distortion, ...
    'Skew', 0.000000);

% load camera frames
camera_points = readmatrix(fullfile("data", exp_name, sprintf("%s.xlsx", camera_name)));
% load radar frames
radar_points = readmatrix(fullfile("data", exp_name, sprintf("%s.xlsx", radar_name)));
% find valid pairs
mask_camera_valid = ~(sum(camera_points == 0, 2)==size(camera_points, 2));
mask_radar_valid = ~(sum(radar_points == 0, 2)==size(radar_points, 2));
mask_valid = mask_camera_valid & mask_radar_valid;
n_pairs = sum(mask_valid);
camera_points = camera_points(mask_valid, :);
radar_points = radar_points(mask_valid, :);
% undistort points
camera_points_undistorted = undistortPoints(camera_points, intrinsics);

% Defind x
x=[sym('thetax');sym('thetay');sym('thetaz');sym('tx');sym('ty');sym('tz')];

% Init x
xk=[0;0;0;0;0;0];

% Construct extrinsic matrix
Rx=[1,0,0;0,cos(x(1)),sin(x(1));0,-sin(x(1)),cos(x(1))];
Ry=[cos(x(2)),0,-sin(x(2));0,1,0;sin(x(2)),0,cos(x(2))];
Rz=[cos(x(3)),sin(x(3)),0;-sin(x(3)),cos(x(3)),0;0,0,1];
B=[Rx*Ry*Rz,[x(4);x(5);x(6)]];

% Construct transform matrix
A = intrinsics.K;
H = A*B;

% Construct optimization function's by calculate projection error
xyz1 = [radar_points, ones(n_pairs, 1)];
uvz = xyz1 * H';
uv = uvz(:, 1:2) ./ uvz(:, 3);
f = camera_points_undistorted - uv;
f = reshape(f, [], 1);

%L-M算法部分，求解非线性优化问题，优化变量x。
Jf = jacobian(f);%计算函数雅可比矩阵（double float）
xk_ini = xk;%优化变量赋初值（double float）
fk = double(subs(f,x,xk));%f函数赋值（double float）
Jfk = double(subs(Jf,x,xk));%Jf函数赋值（double float）
Dk = sqrt(diag(diag(Jfk'*Jfk)));%Dk赋初值（double float）
sigma = 0.1;%sigma参数设置（float）
pk = 1;%Pk赋初值（double float）
deltak = 0.1;%步长赋初值（float）
i = 0;%循环计数（uint16）
% while norm(pk)>0.1 %另一优化阈值 
while i<200 %以循环数作为优化阈值，优化阈值设置。
    i = i+1%循环计数 
    %step a,计算lambda：牛顿法权重参数
    Jfkpinv=pinv(Jfk);%求伪逆（double float）
    if norm(Dk*pinv(Jfk)*fk)<=(1+sigma)*deltak%判断下降梯度
        %梯度平稳，线性近似
        lambdak=0;%（double float）
        pk=-pinv(Jfk)*fk;%变化量pk（double float）
    else
        %梯度过陡，牛顿法拟合
        alpha=0;%lambda优化初值（double float）
        u=norm((Jfk*inv(Dk))'*fk)/deltak;%上确界计算（double float）
        palpha=psolution(alpha,fk,Jfk,Dk);%p_alpha计算(double float）
        qalpha=Dk*palpha;%q_alpha计算(double float）
        phi=norm(qalpha)-deltak;%phi计算(double float）
        dphi = dphisolution(alpha,fk,Jfk,Dk);%dphi/dlambda计算(double float）
        l=-phi/dphi;%下确界计算(double float）
        j=0;%循环初始化（uint16）
        while (abs(phi)>sigma*deltak)&&(j<100)%lambda优化循环
            j=j+1;%循环计数
            if alpha<=l||alpha>=u%判断是否超过取值范围
                alpha=(0.001*u>sqrt(l*u))*0.001*u+(0.001*u<=sqrt(l*u))*sqrt(l*u);%超过时对优化变量alpha进行调整
            end
            if phi<0%判断是否下降过少
                u=alpha;%上确界更新
            end
            l=l*(l>(alpha-phi/dphi))+(alpha-phi/dphi)*(l<=(alpha-phi/dphi));%下确界更新
            alpha=alpha-(phi+deltak)/deltak*phi/dphi;%alpha更新
            palpha=psolution(alpha,fk,Jfk,Dk);%p_alpha更新
            qalpha=Dk*palpha;%q_alpha更新
            phi=norm(qalpha)-deltak;%phi更新
            dphi=dphisolution(alpha,fk,Jfk,Dk);%dphi/dlambda更新
        end
        lambdak=alpha;%优化完成，lambda赋值
        pk = psolution(lambdak,fk,Jfk,Dk);%pk计算
    end
    %step b，步长评估，线性度计算
    fkp=double(subs(f,x,xk+pk));%变化后优化函数取值(double float）
    fkkp(:,i)=fkp'*fkp;
    rhok=((fk'*fk)-fkp'*fkp)/(fk'*fk-(fk+Jfk*pk)'*(fk+Jfk*pk)) ;%线性度计算(double float）
    %step c，优化变量更新
    if rhok>0.0001%线性度合适
        xk=xk+pk;%优化变量更新
        fk=double(subs(f,x,xk));%fk更新
        Jfk=double(subs(Jf,x,xk)); %Jfk更新
    end
    %step d，步长更新
    if rhok<=0.25%线性度过小，说明步长过大
        deltak=0.5*deltak;%步长缩小调整
    elseif (rhok>0.25&&rhok<0.75&&lambdak==0)||rhok>=0.75%线性度过大，说明步长过小
        deltak=2*norm(Dk*pk);%步长扩大调整
    end
    %step e，更新步长梯度
    Dk=(Dk>sqrt(diag(diag(Jfk'*Jfk)))).*Dk+(Dk<=sqrt(diag(diag(Jfk'*Jfk)))).*sqrt(diag(diag(Jfk'*Jfk)));%更新Dk
    xkk(:,i)=xk;%优化变量过程量存储，12×I矩阵，I为优化循环数（目前为1000）
   
end

xkk_ini = xk_ini*ones(1,size(xkk,2));%迭代精度计算中所用矩阵换算（12×I矩阵，double float）
y = diag(sqrt((xkk-xkk_ini)'*(xkk-xkk_ini)));%迭代精度计算，均方误差（double float）
figure(1);
plot(y);
xlabel("step");
ylabel("mse");

% calculate errors
errors = reshape(double(subs(f,x,xk)), [], 2);
errors_u = abs(errors(:, 1)); 
errors_v = abs(errors(:, 2)); 
errors_uv = sqrt(sum(errors.^2, 2));

figure(2);
subplot(2, 2, [1, 2]);
bar(errors_uv);
hold on;
line([1, length(errors_uv)], [mean(errors_uv), mean(errors_uv)],'linewidth',2,'color', 'r');
text(length(errors_uv), mean(errors_uv), ...
    sprintf('%.2f pixels', mean(errors_uv)), "Color", "red", ...
    "HorizontalAlignment", "left", "VerticalAlignment", "bottom");
hold off;
grid on;
xlabel('id_pair', "Interpreter", "none");
ylabel('errors_uv', "Interpreter", "none");

subplot(2, 2, 3);
bar(errors_u);
hold on;
line([1, length(errors_u)], [mean(errors_u), mean(errors_u)],'linewidth',2,'color', 'r');
text(length(errors_u), mean(errors_u), ...
    sprintf('%.2f pixels', mean(errors_u)), "Color", "red", ...
    "HorizontalAlignment", "left", "VerticalAlignment", "bottom");
hold off;
grid on;
xlabel('id_pair', "Interpreter", "none");
ylabel('errors_u', "Interpreter", "none");

subplot(2, 2, 4);
bar(errors_v);
hold on;
line([1, length(errors_v)], [mean(errors_v), mean(errors_v)],'linewidth',2,'color', 'r');
text(length(errors_v), mean(errors_v), ...
    sprintf('%.2f pixels', mean(errors_v)), "Color", "red", ...
    "HorizontalAlignment", "left", "VerticalAlignment", "bottom");
hold off;
grid on;
xlabel('id_pair', "Interpreter", "none");
ylabel('errors_v', "Interpreter", "none");

savefig(fullfile(output_folder, sprintf("%s_to_%s_error.fig", radar_name, camera_name)));

%% show project pcd on image
if show_on    
    image_folder = fullfile("data", exp_name, camera_name);
    images = dir(image_folder);
    images = sort_nat({images.name});
    images = images(3:end)';
    images = images(mask_valid);
    
    figure(3);
    for i = 1:n_pairs
        image_path = fullfile(image_folder, images{i});
        image = imread(image_path);
        image_undistort = undistortImage(image, intrinsics);
        project_points = double(subs(uv,x,xk));
        project_points = round(project_points);

        imshow(image_undistort);
        hold on;
        scatter(camera_points_undistorted(i, 1), camera_points_undistorted(i, 2), 'filled', 'green');
        text(camera_points_undistorted(i, 1), camera_points_undistorted(i, 2), ...
            "gt point", "Color", "green", "VerticalAlignment", "top");
        scatter(project_points(i, 1), project_points(i, 2), 'filled', 'red');
        text(project_points(i, 1), project_points(i, 2), "projected point", "Color", "red", ...
            "HorizontalAlignment", "right", "VerticalAlignment", "top");
        hold off;
        title(sprintf("%s\nerror_uv=%.2f pixels, error_u=%.2f pixels, error_v=%.2f pixels", ...
            image_path, errors_uv(i), errors_u(i), errors_v(i)), ...
            "Interpreter", "none");
    end    
end

% save as json
extrinsic_matrix = double(subs(B,x,xk));
json_data = struct("extrinsic_matrix", extrinsic_matrix);
json_path = fullfile(output_folder, sprintf("%s_to_%s_extrinsic.json", radar_name, camera_name));
savejson('', json_data, json_path);