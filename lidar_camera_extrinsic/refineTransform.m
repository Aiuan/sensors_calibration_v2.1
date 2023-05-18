function [tform, fval, n_use]  = refineTransform(lidarCorners, imageCorners, initialTransformation, fval_thred, n_thred)
    % Extract normals of checkerboard in images
    imNew = reshape(permute(imageCorners, [2 1 3]), size(imageCorners, 2), [])';
    lic = reshape(permute(lidarCorners, [2 1 3]), size(lidarCorners, 2), [])';
    [tform, ~, fval] = pcregistericp(pointCloud(lic), pointCloud(imNew),  ...
        'InitialTransform', initialTransformation, 'MaxIterations', 1000);
    % Filter frames with high errors
    while fval > fval_thred && size(lidarCorners, 3) >= n_thred
        error = computeError(lidarCorners, imageCorners, tform);
        [~, index] = max(error);
        %index = randi(size(lidarCorners, 3), 1);
        imageCorners(:, :, index) = [];
        lidarCorners(:, :, index) = [];
        imNew = reshape(permute(imageCorners, [2 1 3]), size(imageCorners, 2), [])';
        lic = reshape(permute(lidarCorners, [2 1 3]), size(lidarCorners, 2), [])';
        [tform, ~, fval] = pcregistericp(pointCloud(lic), pointCloud(imNew),...
            'InitialTransform', tform, 'MaxIterations', 1000);
    end
    n_use = size(imageCorners, 3);
end

%-------------------------------------------------------------------
function error = computeError(d1, d2, tform)
    transformedCorners = transformLidar3DCorners(d1, tform);
    m1 = mean(transformedCorners);
    m1 = reshape(m1, 3, [])';
    
    m2 = mean(d2);
    m2 = reshape(m2, 3, [])';
    
    transDiff = (m1 - m2);
    error = sqrt(transDiff(:,1).^2 + transDiff(:,2).^2 + transDiff(:,3).^2);
end

%-------------------------------------------------------------------
function lidar3DCorners = transformLidar3DCorners(lidar3DCorners, initialTform)
    corners = reshape(permute(lidar3DCorners, [2 1 3]), size(lidar3DCorners, 2), [])';
    ptCloud = pctransform(pointCloud(corners), initialTform);
    lidar3DCorners = permute(reshape(ptCloud.Location', 3,4,[]),[2,1,3]);
end