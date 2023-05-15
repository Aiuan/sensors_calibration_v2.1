'''
output csv format:
    gps_time,x,y,z,ve(m/s),vn(m/s),vu(m/s),roll(rad),pitch(rad),yaw(rad)
    2021-10-25-06-48-46-030,-2942893.059738322,4679952.56010263,3170232.8318430767,-3.2712,-6.6358,0.0991,0.029207947151114196,0.013821517862745587,3.6040406772518168
'''

import time

import numpy as np
import pandas as pd

from MEMS.asc_decoder import MEMSDecoder

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

def main():
    MEMS_asc_path = 'F:\\sensors_calibration_v2\\rawdata\\20221217_calibration\\MEMS_egoheading_calibration\\MEMS\\NMUT21160006Z_2022-12-17_08-44-11.ASC'
    output_csv_path = './runs/20221217_zhoushan/heading_imu_input.csv'

    md = MEMSDecoder(MEMS_asc_path)

    df = pd.DataFrame(
        columns=['gps_time', 'x', 'y', 'z', 've(m/s)', 'vn(m/s)', 'vu(m/s)', 'roll(rad)', 'pitch(rad)', 'yaw(rad)']
    )

    ts_unique = np.unique(md.data['ts_unix'].values)

    # ref
    ts_str = '{:.3f}'.format(ts_unique[0])
    msg_imu, msg_ins, error, ts_str_MEMS = md.select_by_ts(ts_str)
    lat_ref = msg_ins.latitude_N
    lon_ref = msg_ins.longitude_E
    h_ref = msg_ins.height

    for i in range(len(ts_unique)):
        ts_str = '{:.3f}'.format(ts_unique[i])
        msg_imu, msg_ins, error, ts_str_MEMS = md.select_by_ts(ts_str)

        if msg_ins.ins_status != 'INS_SOLUTION_GOOD':
            print('msg_ins.ins_status = {}'.format(msg_ins.ins_status))

        if msg_ins.pos_type != 'INS_RTKFIXED':
            print('msg_ins.pos_type = {}'.format(msg_ins.pos_type))

        # local time
        ts = float(ts_str)
        local_time_str = time.strftime(
            '%Y-%m-%d-%H-%M-%S', time.localtime(ts)
        ) + '-{:>03d}'.format(round((ts - int(ts)) * 1e3))

        # latitude, longitude, height --> x_ecef, y_ecef, z_ecef
        lat = msg_ins.latitude_N
        lon = msg_ins.longitude_E
        h = msg_ins.height
        x, y, z = pos2enu(lon, lat, h, lon_ref, lat_ref, h_ref)

        # 原始标定程序坐标系：车辆和组合导航皆为右手系，x轴指向前方，y轴指向左方
        # 我们实验所定义的坐标系：车辆和组合导航皆为右手系，x轴指向右方，y轴指向前方
        # 利用做小二乘法拟合ax + by + c =0
        # 修改标定程序249行: (-adjust_a / adjust_b)为(-adjust_b / adjust_a)
        df = df.append({
            'gps_time': local_time_str,
            'x': x,
            'y': y,
            'z': z,
            've(m/s)': msg_ins.east_vel,
            'vn(m/s)': msg_ins.north_vel,
            'vu(m/s)': msg_ins.up_vel,
            'roll(rad)': msg_ins.roll / 180 * np.pi,
            'pitch(rad)': msg_ins.pitch / 180 * np.pi,
            'yaw(rad)': msg_ins.azimuth / 180 * np.pi,
        }, ignore_index=True)

        print('({}/{}) record done'.format(i+1, len(ts_unique)))

    df.to_csv(output_csv_path, sep=',', header=True, index=False)
    print('Generate {} successfully'.format(output_csv_path))

if __name__ == '__main__':
    main()
