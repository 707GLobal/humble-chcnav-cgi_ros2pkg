# NovAtel OEM7 28 字节帧头 (H=28)
# 同步字 AA 44 12 1C 固定，不在此消息中保存
uint8   header_len      # byte3:  帧头长度，OEM7 = 28 (0x1C)
uint16  msg_id          # byte4-5: 消息号，小端，BESTPOS=42 HEADING=971
uint8   msg_type        # byte6:  0=二进制 1=ASCII 2=简写ASCII 3=NMEA
uint8   port_addr       # byte7:  端口地址
uint16  msg_length      # byte8-9: 消息长度（不含CRC），小端
uint16  sequence        # byte10-11: 消息序号
uint8   idle_time       # byte12: 空闲时间
uint8   time_status     # byte13: 时间状态
uint16  gps_week        # byte14-15: GPS 周
uint32  gps_ms          # byte16-19: GPS 周内毫秒
uint32  receiver_status # byte20-23: 接收机状态
uint16  reserved        # byte24-25: 预留
uint16  sw_version      # byte26-27: 接收机软件版本
