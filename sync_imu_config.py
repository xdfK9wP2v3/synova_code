import socket

save_msg = "SAVCFG"
rst_msg = "RSTALL"
bkurst_msg = "BKURST"

# CH1: TMS, 1Hz
tms_timec = "CONFIG,TIMEC,1,11999,0,N"
tms_trigr = "CONFIG,TRIGR,1,1999,0,0,0000000000000000,0000000000000000,0000000000000000,0000000000000001,0000000000000000,AAAAAAAAAAAAAAAB,H,DD"

# CH2: ADIS, 2kHz
adis_trig_timec = "CONFIG,TIMEC,2,5,0,N"
adis_trig_trigr = "CONFIG,TRIGR,2,2,0,0,0000000000000000,0000000000000000,0000000000000000,0000000000000002,0000000000000000,AAAAAAAAAAAAAAA8,H,DD"
edge_config = "CONFIG,CAPIN,BRBBBBBBBBBB"

# CH6: Camera, 300Hz
camera_trig_timec = "CONFIG,TIMEC,3,38,0,D"
camera_trig_trigr = "CONFIG,TRIGR,6,9,0,0,0000000000000000,0000000000000000,0000000000000000,0000000000000004,0000000000000000,AAAAAAAAAAAAAAAE,H,DD"

# CH6: Camera, 24Hz
camera_trig_timec_low = "CONFIG,TIMEC,3,498,0,D"
camera_trig_trigr_low = "CONFIG,TRIGR,6,99,0,0,0000000000000000,0000000000000000,0000000000000000,0000000000000004,0000000000000000,AAAAAAAAAAAAAAAE,H,DD"

# CH7 (ego): Radar, 1Hz, shift 0
radar_ego_trig_timec = "CONFIG,TIMEC,4,11999,0,N"
radar_ego_trig_trigr = "CONFIG,TRIGR,7,2999,0,0,0000000000000000,0000000000000000,0000000000000000,0000000000000008,0000000000000000,AAAAAAAAAAAAAAA2,H,DD"

# CH7 (rsu): Radar, 1Hz, shift 36us / 2
radar_rsu_trig_timec = "CONFIG,TIMEC,5,11999,0,N"
radar_rsu_trig_trigr = "CONFIG,TRIGR,7,2999,0,-4320,0000000000000000,0000000000000000,0000000000000000,000000000000010,0000000000000000,AAAAAAAAAAAAAABA,H,DD"

# CH8: Lidar, 1Hz
lidar_sec_timec = "CONFIG,TIMEC,6,11999,0,N"
lidar_sec_trigr = "CONFIG,TRIGR,8,3999,0,0,0000000000000000,0000000000000000,0000000000000000,000000000000020,0000000000000000,AAAAAAAAAAAAAA8A,H,DD"

# CH9 (ego): Radar (MIMO), 10Hz
radar_trig_timec_mimo = "CONFIG,TIMEC,7,1199,0,N"
radar_trig_trigr_mimo = "CONFIG,TRIGR,9,299,0,-6000,0000000000000000,0000000000000000,0000000000000000,0000000000000040,0000000000000000,AAAAAAAAAAAAAAEA,H,DD"

# CH10: Camera 2, 24Hz, timec 3
camera2_trig_trigr_low = "CONFIG,TRIGR,10,99,0,0,0000000000000000,0000000000000000,0000000000000000,0000000000000004,0000000000000000,AAAAAAAAAAAAAAAE,H,DD"

# $CONFIG,RTKB,freset,id,time,distance,pps_type,nmea_bitmap,nmea_freq*
imu_rtkb = "CONFIG,RTKB,P,1,30,4,2,81,0"
# $CONFIG,RTKM,freset (reset R),pps_type,heading_length,heading_error,nmea_bitmap,nmea_freq*
imu_rtkm = "CONFIG,RTKM,P,1,90,5,FF,1"
# $CONFIG,IMUF,adis_sync_mode,mag_freq,lps28_freq,lps22_freq*
imu_imuf = "CONFIG,IMUF,1,80,10,10"

ip_sync = "CONFIG,IPMAC,192.168.012.012,255.255.255.000,192.168.012.001,00:80:E1:01"
ip_imu = "CONFIG,IPMAC,192.168.012.011,255.255.255.000,192.168.012.001,00:80:E1:0A"


def checksum_xor(msg: str) -> str:
    checksum = 0
    msg_bytes = msg.encode()
    for b in msg_bytes:
        checksum = (checksum ^ b) & 0xff

    return f"{checksum:02x}".upper()


def calculate_trig_checksum(num: int) -> None:
    print(f"{num ^ 0xAAAAAAAAAAAAAAAA:02X}")


def config_sync(message: str, server_ip: str) -> None:
    server_port = 6000
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        client_socket.connect((server_ip, server_port))
        print(f"Connected to {server_ip}:{server_port}")

        message = "$" + message + "*" + checksum_xor(message) + "\r\n"
        client_socket.send(message.encode('utf-8'))
        print(f"Sent: {message}")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        client_socket.close()
        print("Connection closed")


def config_imu(message: str, server_ip: str) -> None:
    server_port = 5004
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        client_socket.connect((server_ip, server_port))
        print(f"Connected to {server_ip}:{server_port}")

        message = "$" + message + "*" + checksum_xor(message) + "\r\n"
        client_socket.send(message.encode('utf-8'))
        print(f"Sent: {message}")

    except Exception as e:
        print(f"Error: {e}")

    finally:
        client_socket.close()
        print("Connection closed")


def sync_channel_config(ip: str) -> None:
    config_sync(tms_timec, ip)
    config_sync(tms_trigr, ip)

    config_sync(adis_trig_timec, ip)
    config_sync(adis_trig_trigr, ip)

    config_sync(camera_trig_timec, ip)
    config_sync(camera_trig_trigr, ip)

    if ip.split('.')[-2] == '12':
        config_sync(radar_rsu_trig_timec, ip)
        config_sync(radar_rsu_trig_trigr, ip)
    else:
        config_sync(radar_ego_trig_timec, ip)
        config_sync(radar_ego_trig_trigr, ip)
        config_sync(radar_trig_timec_mimo, ip)
        config_sync(radar_trig_trigr_mimo, ip)

    config_sync(lidar_sec_timec, ip)
    config_sync(lidar_sec_trigr, ip)

    config_sync(edge_config, ip)
    config_sync(save_msg, ip)


def imu_params_config(ip: str, rtk_type: str):
    if rtk_type == "BS":
        config_imu(imu_rtkb, ip)
    elif rtk_type == "MOB":
        config_imu(imu_rtkm, ip)
    else:
        raise Exception(f"Unknown RTK type: {rtk_type}")

    config_imu(imu_imuf, ip)
    config_imu(save_msg, ip)
