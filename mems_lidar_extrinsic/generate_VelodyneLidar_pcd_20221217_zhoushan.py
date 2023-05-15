from VelodyneLidar.decoder import *

def main():
    config_path = 'VelodyneLidar/Alpha Prime.xml'
    data_path = 'F:\\sensors_calibration_v2\\rawdata\\20221217_calibration\\MEMS_lidar_calibration\\VelodyneLidar\\gps_lidar_calib_20221217.pcap'
    output_path = 'F:\\sensors_calibration_v2\\rawdata\\20221217_calibration\\MEMS_lidar_calibration\\VelodyneLidar_pcd'

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