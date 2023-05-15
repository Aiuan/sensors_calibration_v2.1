function [errors_translation, errors_rotation, errors_reprojection] = computeErrors( ...
    lidarCorners3d, imageCorners3d, cameraParams, tform)

    % Transform Lidar corners to camera frame
    transformedCorners = transformLidar3DCorners(lidarCorners3d, tform);

    % Find the centroids of the corners
    lidarCentroids = mean(transformedCorners);
    imageCentroids = mean(imageCorners3d);

    % Reshape in n-by-3 matrix
    lidarCentroids = reshape(lidarCentroids, 3, [])';
    imageCentroids = reshape(imageCentroids, 3, [])';

    % Find the difference between the centroid
    transDiff = lidarCentroids - imageCentroids;

    % Compute Translation Error
    errors_translation = sqrt(transDiff(:,1).^2 + transDiff(:,2).^2 + transDiff(:,3).^2);

    % Compute Rotation Error
    errors_rotation = computeRotationError(transformedCorners, imageCorners3d);

    % Compute Reprojection Error with camera intrinsic is provided
    errors_reprojection = computeReprojectionError(transformedCorners, imageCorners3d, cameraParams);
end

function lidar3DCorners = transformLidar3DCorners(lidar3DCorners, initialTform)
    lidar3DCorners = reshape(permute(lidar3DCorners, [2, 1, 3]), size(lidar3DCorners, 2), [])';
    lidar3DCorners = lidar3DCorners*initialTform.Rotation + initialTform.Translation;
    lidar3DCorners = permute(reshape(lidar3DCorners', 3, 4, []), [2, 1, 3]);
end

function rotationError = computeRotationError(lidarData, imageData)
    numFrames = size(lidarData, 3);
    rotationError = zeros(numFrames, 1, 'like', imageData);
    
    % Extract Lidar and image plane and edge parameters
    [~, lidarPlaneParameters] = lidar.internal.calibration.computePlaneAndEdgeParameters(lidarData);
    [~, imagePlaneParameters] = lidar.internal.calibration.computePlaneAndEdgeParameters(imageData);
    
    for i = 1:numFrames
        c1 = mean(lidarData(:, :, i));
        c2 = mean(imageData(:, :, i));
        n1 = lidarPlaneParameters(i, 1:3);
        n2 = imagePlaneParameters(i, 1:3);
    
        % Ensure the normals are facing towards the sensor
        n1 = correctNormal(n1, c1);
        n2 = correctNormal(n2, c2);
    
        % Compute the difference between normals
        normalTheta = rad2deg(acos(dot(n1, n2/(norm(n1) * norm(n2)))));
        if normalTheta > 90
            normalTheta = 180 - normalTheta;
        end
        rotationError(i) = normalTheta;    
    end
end

function normal = correctNormal(normal, centroid)
    sensorCenter = [0,0,0];
    p1 = sensorCenter - centroid;
    p2 = normal;
    angle = atan2(norm(cross(p1,p2)),p1*p2');
    if angle > pi/2 || angle < -pi/2
        normal = -normal;
    end
end

function repoError = computeReprojectionError(lidarCorners3d, imageCorners3d, cameraParams)
    %   Compute reprojection error between the projected lidar centroids and image centroids
    lidarData = reshape(permute(lidarCorners3d, [1,3,2]), [], 3);
    imageData = reshape(permute(imageCorners3d, [1,3,2]), [], 3);    
    projectedLidarPts = lidar.internal.calibration.projectLidarToCamera(pointCloud(lidarData), cameraParams, rigidtform3d());
    projectedImagePts = lidar.internal.calibration.projectLidarToCamera(pointCloud(imageData), cameraParams, rigidtform3d());
    repo = (projectedLidarPts - projectedImagePts);
    repoError = sqrt(repo(:,1).^2 + repo(:,2).^2);
    repoError = reshape(repoError, 4, [])';
    repoError = mean(repoError, 2);
end