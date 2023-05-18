function extrinsic = load_extrinsic(path)
    json_data = loadjson(path);

    extrinsic = rigidtform3d(json_data.extrinsic_matrix);
end