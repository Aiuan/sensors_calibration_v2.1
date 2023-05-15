function initialTransformation = computeInitialTransform(lidarCorners, imageCorners)
    %computeInitialTransform Estimates initial transformation between Lidar and
    %   camera using checkerboard corners from lidar point cloud and camera
    %   images
    %
    %   initialTransformation = computeInitialTransform(lidarCorners, imageCorners)
    %   computes initialTransformation, an rigid3d object, using lidarCorners,
    %   a 4-by-3-by-P matrix containing four corners for checkerboard from
    %   lidar point cloud. imageCorners is a 4-by-3-by-P matrix containing four
    %    corners for checkerboard from camera images. P is the number of
    %    frames.
    %
    
    %   Copyright 2019-2021 The MathWorks, Inc.
    %
    %   References
    %
    %   [1] Lipu Zhou and Zimo Li and Michael Kaess,
    %   "Automatic Extrinsic Calibration of a Camera and a 3D LiDAR using Line
    %   and Plane Correspondences. Intl. Conf. on Intelligent Robots and
    %   Systems, IROS, Oct 2018
    
    [lidarEdgeDirection, lidarPlaneParameters] = lidar.internal.calibration.computePlaneAndEdgeParameters(lidarCorners);
    [imageEdgeDirection, imagePlaneParameters] = lidar.internal.calibration.computePlaneAndEdgeParameters(imageCorners);
    
    % Get initial rotation matrix
    initialRotation = computeInitialRotation(lidarEdgeDirection, imageEdgeDirection, ...
        imagePlaneParameters(:,1:3), lidarPlaneParameters(:, 1:3));
    initialTranslation = computeInitialTranslation(imageEdgeDirection, ...
        imagePlaneParameters, lidarCorners, imageCorners, initialRotation);
    
    initialTransformation = rigidtform3d(initialRotation', initialTranslation);
end

%--------------------------------------------------------------------------
function coarseRotation = computeInitialRotation(lidarEdgeDirection, imageEdgeDirection, ...
    imageNormals, lidarNormals)
    %computeInitialRotation Estimates initial rotation matrix
    
    numPatterns = size(lidarEdgeDirection,3);
    % reshape normals from Px3 to 1x3xP matrix
    cameraNormals = reshape(imageNormals',1,3,[]);
    lidarNormals = reshape(lidarNormals',1,3,[]);
    Ml = zeros(5,3,numPatterns);
    Mc = zeros(5,3,numPatterns);
    
    Ml(1,:,:)=lidarNormals;
    Ml(2:5,:,:) = lidarEdgeDirection;
    Mc(1,:,:)=cameraNormals;
    Mc(2:5,:,:) = imageEdgeDirection;
    Ml = reshape(permute(Ml, [2 1 3]), size(Ml, 2), [])';
    Mc = reshape(permute(Mc, [2 1 3]), size(Mc, 2), [])';
    [U,~,V] = svd(Ml'*Mc);
    % Use det(V*U') to handle degenerate case arising
    % due to reflection matrix
    refMatrix = [1, 0, 0; 0, 1, 0; 0, 0, sign(det(V*U'))];
    coarseRotation = V*refMatrix*U'; % Camera to Lidar
    % Convert transformation to lidar to camera
    coarseRotation = coarseRotation';
end

%--------------------------------------------------------------------------
function coarseTranslation = computeInitialTranslation(imageEdgeDirection, ...
    imagePlaneParameters, lidarCorners, imageCorners, coarseRotation)
    %computeInitialTranslation Estimates initial translation matrix
    
    numPatterns = size(imageEdgeDirection, 3);
    % Extract normals
    imageNormal = imagePlaneParameters(:, 1:3);
    
    % Compute A matrix
    [Ablock, A] = lidar.internal.calibration.computeAMatrix(imageEdgeDirection);
    
    % Create rigid3d object
    R = [[coarseRotation; [0, 0, 0]], [0; 0; 0; 1]];
    tform = rigid3d(R);
    
    % Get the centroid of the checkerboard plane in Lidar
    centroidLidar = mean(lidarCorners);
    centroidLidar = reshape(centroidLidar, 3, [])';
    
    % Transform the centroid of the lidar
    transformedCentroidLidar = centroidLidar*tform.Rotation + tform.Translation;
    
    
    % nc.tc = -nc.R*Pl -dc
    diffCentroid = -diag(imageNormal*transformedCentroidLidar') - abs(imagePlaneParameters(:, 4));
    
    % Extract mean of edges
    imageEdgeMean = extractEdgeMean(imageCorners);
    lidarEdgeMean = extractEdgeMean(lidarCorners);
    
    lidarEdgeMean = reshape(permute(lidarEdgeMean, [2 1 3]), size(lidarEdgeMean, 2), [])';
    imageEdgeMean = reshape(permute(imageEdgeMean, [2 1 3]), size(imageEdgeMean, 2), [])';
    
    % Transform Lidar mean edges
    lidarEdgeMean = lidarEdgeMean*tform.Rotation + tform.Translation;
    
    diff = lidarEdgeMean - imageEdgeMean;
    c = reshape(diff', 12, []);
    
    rhs = zeros(13, numPatterns);
    lhs = zeros(13, 3);
    for count = 1:numPatterns
        rhs(1,count)  = diffCentroid(count);
        rhs(2:end, count) = -Ablock(:, :, count)*c(:, count);
        lhs(1, :, count)  = imageNormal(count, :);
        lhs(2:end, :, count) = A(:, :, count);
    end
    
    lhs = (reshape(permute(lhs, [2 1 3]), size(lhs, 2), [])');
    rhs = reshape(rhs,[],1);
    coarseTranslation=(rhs'*rhs)\(rhs'*lhs);
end

%--------------------------------------------------------------------------
function meanEdge = extractEdgeMean(corners)
    %extractEdgeMean Calculates the mean of the edge from the corners
    meanEdge = zeros(size(corners));
    for count = 1:size(corners,3)
        meanEdge(1,:,count) = (corners(1,:,count) + corners(2,:,count))/2;
        meanEdge(2,:,count) = (corners(2,:,count) + corners(3,:,count))/2;
        meanEdge(3,:,count) = (corners(3,:,count) + corners(4,:,count))/2;
        meanEdge(4,:,count) = (corners(1,:,count) + corners(4,:,count))/2;
    end
end