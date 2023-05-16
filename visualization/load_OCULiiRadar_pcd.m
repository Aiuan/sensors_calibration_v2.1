function pc = load_OCULiiRadar_pcd(path)
    % x y z doppler snr
    pcd = readmatrix(path, "FileType", "text");

    pc = pointCloud(pcd(:, 1:3), "Intensity", pcd(:, 4));
end