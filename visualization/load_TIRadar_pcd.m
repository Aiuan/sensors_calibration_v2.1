function pc = load_TIRadar_pcd(path)
    % x y z doppler snr intensity noise
    pcd = readmatrix(path, "FileType", "text");
    pcd = pcd(9:end, 1:7);

    pc = pointCloud(pcd(:, 1:3), "Intensity", pcd(:, 4));
end