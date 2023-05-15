function [corners_reorder, cuboidCorners, face_front, face_behind] = extractLidarCorners(plane)
    rectModel = lidar.internal.calibration.fitRectangle3D(plane, 'O', 'YPR', 'Iterations', 30);
    cuboidCorners = rectModel.getCornerPoints();
    
    faces = [
        1,2,3,4; 5,6,7,8;
        4,3,7,8; 1,2,6,5;
        3,2,6,7; 4,1,5,8;        
    ];

    S_faces = zeros(size(faces, 1) / 2, 1);
    for i = 1:size(S_faces, 1) 
        edge1 = sqrt(sum((cuboidCorners(faces(i*2-1, 1), :) - cuboidCorners(faces(i*2-1, 2), :)).^2));
        edge2 = sqrt(sum((cuboidCorners(faces(i*2-1, 2), :) - cuboidCorners(faces(i*2-1, 3), :)).^2));
        S_faces(i) = edge1 * edge2;
    end
    [~, index_S] = max(S_faces);

    face1 = faces(index_S*2-1, :);
    face2 = faces(index_S*2, :);   
    center1 = mean(cuboidCorners(face1, :), 1);
    center2 = mean(cuboidCorners(face2, :), 1);

    if sum(center1.^2) <= sum(center2.^2)
        % clockwise 
        face_front = face1(end:-1:1);
        face_behind = face2(end:-1:1);
        center_front = center1;
        center_behind = center2;
    else
        face_front = face2;
        face_behind = face1;
        center_front = center2;
        center_behind = center1;
    end
        
    corners = (cuboidCorners(face_front, :) + cuboidCorners(face_behind, :)) / 2;
    % select top point as first   
    index_corners = zeros(1, size(corners, 1));
    [~, index_first] = max(corners(:, 3));
    index_corners(1) = index_first;
    for i = 1 : size(corners, 1)-1
        index_next = index_first+i;
        if index_next > size(corners, 1)
            index_next = mod(index_next, size(corners, 1));
        end
        index_corners(i+1) = index_next;
    end

    corners_reorder = corners(index_corners, :);
    

%     figure();
%     hold on;
%     % face_front
%     scatter3(cuboidCorners(face_front, 1), cuboidCorners(face_front, 2), cuboidCorners(face_front, 3), 40, '.r');
%     text(cuboidCorners(face_front, 1), cuboidCorners(face_front, 2), cuboidCorners(face_front, 3), ...
%         split(num2str(face_front)), "Color", 'red', 'HorizontalAlignment', 'center');
%     scatter3(center_front(:, 1), center_front(:, 2), center_front(:, 3), '*r');
%     
%     % face_behind
%     scatter3(cuboidCorners(face_behind, 1), cuboidCorners(face_behind, 2), cuboidCorners(face_behind, 3), 40, '.b');
%     text(cuboidCorners(face_behind, 1), cuboidCorners(face_behind, 2), cuboidCorners(face_behind, 3), ...
%         split(num2str(face_behind)), "Color", 'blue', 'HorizontalAlignment', 'center');
%     scatter3(center_behind(:, 1), center_behind(:, 2), center_behind(:, 3), '*b');
%     
%     % corners
%     scatter3(corners_reorder(:, 1), corners_reorder(:, 2), corners_reorder(:, 3), '.k');
%     text(corners_reorder(:, 1), corners_reorder(:, 2), corners_reorder(:, 3), ...
%         split(num2str(1:size(corners_reorder, 1))), ...
%         "Color", 'black', 'HorizontalAlignment', 'center');
% 
%     hold off;
%     axis("equal");
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
%     grid on;
%     view([20, 0]);
end
