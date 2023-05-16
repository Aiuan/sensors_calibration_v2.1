function pc = load_VelodyneLidar_pcd(path)
    % x y z intensity idx_laser unix_timestamp
    pcd = readmatrix(path, "FileType", "text");

    pc = pointCloud(pcd(:, 1:3), "Intensity", pcd(:, 4));
end