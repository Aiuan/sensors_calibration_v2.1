'''
    PS: 激光雷达所采集pcap文件数据中position package中nmea消息丢失，因此使用接收时间戳
'''

from VelodyneLidar.decoder import *

def main():
    config_path = 'VelodyneLidar/Alpha Prime.xml'

    # data_path = 'F:\\sensors_calibration_v2\\rawdata\\20221226_calibration\\MEMS_lidar_calibration1\\VelodyneLidar\\calib_gpslidar_20221226_1.pcap'
    # output_path = 'F:\\sensors_calibration_v2\\rawdata\\20221226_calibration\\MEMS_lidar_calibration1\\VelodyneLidar_pcd'

    data_path = 'F:\\sensors_calibration_v2\\rawdata\\20221226_calibration\\MEMS_lidar_calibration2\\VelodyneLidar\\calib_gpslidar_20221226_2.pcap'
    output_path = 'F:\\sensors_calibration_v2\\rawdata\\20221226_calibration\\MEMS_lidar_calibration2\\VelodyneLidar_pcd'


    vd = VelodyneDecoder(
        config_path=config_path,
        pcap_path=data_path,
        output_path=output_path,
        num_skip_packets=0,
        recover=True
    )

    t_last = time.time()
    while 1:
        vd.decode_next_packet()
        if vd.judge_jump_cut_degree():
            vd.generate_frame(pcd_file_type='pcd')
            t = time.time()
            print('    {:.2f} s'.format(t - t_last))
            t_last = time.time()


if __name__ == '__main__':
    main()