function intrinsic = load_intrinsic(path)
    json_data = loadjson(path);

    intrinsic = cameraIntrinsics( ...
        [json_data.intrinsic_matrix(1,1), json_data.intrinsic_matrix(2,2)], ...
        [json_data.intrinsic_matrix(1,3), json_data.intrinsic_matrix(2,3)], ...
        json_data.image_size, ...
        'RadialDistortion', json_data.radial_distortion, ...
        'TangentialDistortion', json_data.tangential_distortion, ...
        'Skew', 0.000000);
end