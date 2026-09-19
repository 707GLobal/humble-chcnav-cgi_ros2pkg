# BESTPOSB 消息，NovAtel OEM7 兼容协议，帧总长 104 字节 (28 帧头 + 72 数据 + 4 CRC)
# 字段偏移与协议表一致：H = 28 为帧头长度
std_msgs/Header                 header
ArcHeader                       arc_header
SolutionStatus                  sol_status                      # H+0  解状态 (4B)
PositionOrVelocityType          pos_type                        # H+4  位置类型 (4B)
float64                         lat                             # H+8  纬度 (deg)
float64                         lon                             # H+16 经度 (deg)
float64                         hgt                             # H+24 椭球高 (m)
float32                         undulation                      # H+32 大地水准面差距 (m)
uint32                          datum_id                        # H+36 坐标系ID (61=WGS84)
float32                         lat_stdev                       # H+40 纬度标准差 (m)
float32                         lon_stdev                       # H+44 经度标准差 (m)
float32                         hgt_stdev                       # H+48 高度标准差 (m)
int8[4]                         stn_id                          # H+52 基站ID
float32                         diff_age                        # H+56 差分龄期 (s)
float32                         sol_age                         # H+60 解龄期 (s)
uint8                           num_svs                         # H+64 跟踪卫星数 (NSV1)
uint8                           num_sol_svs                     # H+65 解中使用的卫星数 (NSU1)
uint8                           num_sol_l1_svs                  # H+66 解中使用 L1/E1/B1 卫星数
uint8                           num_sol_multi_svs               # H+67 解中使用多频卫星数
uint8                           reserved                        # H+68 保留
BestExtendedSolutionStatus      ext_sol_stat                     # H+69 扩展解状态
uint8                           reserved1                       # H+70 保留
uint8                           sig_mask                        # H+71 信号掩码
