import os
import time
import shutil
import json

import numpy as np
import pandas as pd
from pypcd import pypcd

from MEMS.asc_decoder import MEMSDecoder
from VelodyneLidar.timestamp_selector import DataFolder as VDF
from utils import *

def pos2ecef(lon, lat, h):
    # WGS84长半轴
    a = 6378137
    # WGS84椭球扁率
    f = 1 / 298.257223563
    # WGS84短半轴
    b = a * (1 - f)
    # WGS84椭球第一偏心率
    e = np.sqrt(a * a - b * b) / a
    # WGS84椭球面卯酉圈的曲率半径
    N = a / np.sqrt(1 - e * e * np.sin(lat * np.pi / 180) * np.sin(lat * np.pi / 180))
    x_ecef = (N + h) * np.cos(lat * np.pi / 180) * np.cos(lon * np.pi / 180)
    y_ecef = (N + h) * np.cos(lat * np.pi / 180) * np.sin(lon * np.pi / 180)
    z_ecef = (N * (1 - (e * e)) + h) * np.sin(lat * np.pi / 180)

    return x_ecef, y_ecef, z_ecef

def pos2enu(lon, lat, h, lon_ref, lat_ref, h_ref):
    x_ecef, y_ecef, z_ecef = pos2ecef(lon, lat, h)
    x_ecef_ref, y_ecef_ref, z_ecef_ref = pos2ecef(lon_ref, lat_ref, h_ref)

    offset_x, offset_y, offset_z = x_ecef - x_ecef_ref, y_ecef - y_ecef_ref, z_ecef - z_ecef_ref

    sinLon = np.sin(lon_ref * np.pi / 180)
    cosLon = np.cos(lon_ref * np.pi / 180)
    sinLat = np.sin(lat_ref * np.pi / 180)
    cosLat = np.cos(lat_ref * np.pi / 180)

    x_enu = -1 * sinLon * offset_x + cosLon * offset_y
    y_enu = -1 * sinLat * cosLon * offset_x - 1 * sinLat * sinLon * offset_y + cosLat * offset_z
    z_enu = cosLat * cosLon * offset_x + cosLat * sinLon * offset_y + sinLat * offset_z

    return x_enu, y_enu, z_enu

def cosd(degree):
    return np.cos(degree / 180 * np.pi)

def sind(degree):
    return np.sin(degree / 180 * np.pi)

def b2n(pitch, roll, yaw, tx, ty, tz):
    # Z-X-Y
    return np.array([
        [cosd(roll)*cosd(yaw)-sind(pitch)*sind(roll)*sind(yaw), -cosd(pitch)*sind(yaw), cosd(yaw)*sind(roll)+cosd(roll)*sind(pitch)*sind(yaw), tx],
        [cosd(roll)*sind(yaw) + cosd(yaw)*sind(pitch)*sind(roll), cosd(pitch)*cosd(yaw), sind(roll)*sind(yaw)-cosd(roll)*cosd(yaw)*sind(pitch), ty],
        [-cosd(pitch)*sind(roll), sind(pitch), cosd(pitch)*cosd(roll), tz],
        [0, 0, 0, 1]
    ])

def create_init_extrinsic(output_json_path, init_extrinsic):
    dict_data = {
        'gnss-to-top_center_lidar-extrinsic':{
            'param_type': 'extrinsic',
            'sensor_name': 'gnss',
            'target_sensor_name': 'top_center_lidar',
            'device_type': 'relational',
            'param': {
                'sensor_calib': {
                    'data': init_extrinsic,
                    'continuous': True,
                    "rows": 4,
                    "type": 6,
                    "cols": 4
                },
                'time_lag': 0
            }
        }
    }

    with open(output_json_path, 'w') as f:
        json.dump(dict_data, f)

    log_GREEN('Generate {}'.format(output_json_path))

def main():
    VelodyneLidar_pcd_path = 'F:\\sensors_calibration_v2\\rawdata\\20221217_calibration\\MEMS_lidar_calibration\\VelodyneLidar_pcd'
    output_pcd_root = './data/20221217_zhoushan/top_center_lidar'

    MEMS_asc_path = 'F:\\sensors_calibration_v2\\rawdata\\20221217_calibration\\MEMS_lidar_calibration\\MEMS\\NMUT21160006Z_2022-12-17_08-40-59.ASC'
    output_txt_path = './data/20221217_zhoushan/NovAtel-pose-lidar-time.txt'

    output_json_path = './data/gnss-to-top_center_lidar-extrinsic.json'
    init_extrinsic = [
        [1, 0, 0, -0.1],
        [0, 1, 0, 0.01],
        [0, 0, 1, -0.4],
        [0, 0, 0, 1]
    ]
    create_init_extrinsic(output_json_path, init_extrinsic)

    if not os.path.exists(output_pcd_root):
        os.mkdir(output_pcd_root)
        log_GREEN('create {}'.format(output_pcd_root))

    md = MEMSDecoder(MEMS_asc_path)
    vdf = VDF(VelodyneLidar_pcd_path)

    df = pd.DataFrame()

    ts_unique = np.unique(md.data['ts_unix'].values)

    # enu ref
    ts_str = '{:.3f}'.format(ts_unique[0])
    msg_imu, msg_ins, error, ts_str_MEMS = md.select_by_ts(ts_str)
    lat_ref = msg_ins.latitude_N
    lon_ref = msg_ins.longitude_E
    h_ref = msg_ins.height

    for i in range(len(ts_unique)):
        ts_str = '{:.3f}'.format(ts_unique[i])
        msg_imu, msg_ins, error_MEMS, ts_str_MEMS = md.select_by_ts(ts_str)
        pcd_path, error_VelodyneLidar, ts_str_VelodyneLidar = vdf.select_by_ts(ts_str)
        assert error_MEMS == 0
        if error_VelodyneLidar > 0.01:
            log_YELLOW('({}/{}) error_VelodyneLidar={:.3f}s, skip'.format(i+1, len(ts_unique), error_VelodyneLidar))
            continue

        if msg_ins.ins_status != 'INS_SOLUTION_GOOD':
            log_YELLOW('msg_ins.ins_status = {}'.format(msg_ins.ins_status))

        if msg_ins.pos_type != 'INS_RTKFIXED':
            log_YELLOW('msg_ins.pos_type = {}'.format(msg_ins.pos_type))

        # local time
        ts = float(ts_str_MEMS)
        local_time_str = time.strftime(
            '%Y-%m-%d-%H-%M-%S', time.localtime(ts)
        ) + '-{:>03d}'.format(round((ts - int(ts)) * 1e3))

        # latitude, longitude, height --> x_ecef, y_ecef, z_ecef
        lat = msg_ins.latitude_N
        lon = msg_ins.longitude_E
        h = msg_ins.height
        x, y, z = pos2enu(lon, lat, h, lon_ref, lat_ref, h_ref)

        df = df.append({
            'gps_time': local_time_str,
            'unix_ts_MEMS': ts_str_MEMS,
            'error_MEMS': error_MEMS,
            'unix_ts_VelodyneLidar': ts_str_VelodyneLidar,
            'error_VelodyneLidar': error_VelodyneLidar,
            'pcd_path': pcd_path,
            'x': x,
            'y': y,
            'z': z,
            'roll(degree)': msg_ins.roll,
            'pitch(degree)': msg_ins.pitch,
            'azimuth(degree)': msg_ins.azimuth,
        }, ignore_index=True)

        log_GREEN('({}/{}) record done'.format(i+1, len(ts_unique)))


    df_res = pd.DataFrame()

    b2n_0 = b2n(
        pitch=df['pitch(degree)'].iloc[0],
        roll=df['roll(degree)'].iloc[0],
        yaw=-df['azimuth(degree)'].iloc[0] if df['azimuth(degree)'].iloc[0] <= 180 else 360 - df['azimuth(degree)'].iloc[0],
        tx=df['x'].iloc[0],
        ty=df['y'].iloc[0],
        tz=df['z'].iloc[0]
    )

    for i in range(df.shape[0] - 1):
        gps_time = df['gps_time'].iloc[i+1]

        b2n_t = b2n(
            pitch=df['pitch(degree)'].iloc[i+1],
            roll=df['roll(degree)'].iloc[i+1],
            yaw=-df['azimuth(degree)'].iloc[i+1] if df['azimuth(degree)'].iloc[i+1] <= 180 else 360 - df['azimuth(degree)'].iloc[i+1],
            tx=df['x'].iloc[i+1],
            ty=df['y'].iloc[i+1],
            tz=df['z'].iloc[i+1]
        )

        t20 = np.matmul(
            np.linalg.inv(b2n_t),
            b2n_0
        )

        df_res = df_res.append({
            'gps_time': gps_time,
            'Ti00': t20[0, 0],
            'Ti01': t20[0, 1],
            'Ti02': t20[0, 2],
            'Ti03': t20[0, 3],
            'Ti10': t20[1, 0],
            'Ti11': t20[1, 1],
            'Ti12': t20[1, 2],
            'Ti13': t20[1, 3],
            'Ti20': t20[2, 0],
            'Ti21': t20[2, 1],
            'Ti22': t20[2, 2],
            'Ti23': t20[2, 3],
        }, ignore_index=True)

        # rename feilds in pcd
        src = df['pcd_path'].iloc[i+1]
        pc = pypcd.PointCloud.from_path(src)
        pc.fields = ['x', 'y', 'z', 'intensity', 'ring', 'timestamp']
        pc.size = [4, 4, 4, 4, 2, 8]
        pc.type = ['F', 'F', 'F', 'F', 'U', 'F']

        dst = os.path.join(output_pcd_root, '{}.pcd'.format(gps_time))
        pc.save_pcd(dst, compression='ascii')


        log_GREEN('({}/{}) done'.format(i + 1, df.shape[0] - 1))

    df_res.to_csv(output_txt_path, sep=' ', header=False, index=False)
    log_GREEN('Generate {} successfully'.format(output_txt_path))


if __name__ == '__main__':
    main()
