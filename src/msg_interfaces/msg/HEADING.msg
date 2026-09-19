# HEADINGB 消息，NovAtel OEM7 兼容协议，帧总长 76 字节 (28 帧头 + 44 数据 + 4 CRC)
# 字段偏移与协议表一致：H = 28 为帧头长度
std_msgs/Header             header
ArcHeader                   arc_header
SolutionStatus              sol_status                      # H+0  解状态 (4B)
PositionOrVelocityType      pos_type                        # H+4  位置/定向类型 (4B)
float32                     length                          # H+8  基线长 (m, 0~3000)
float32                     heading                         # H+12 航向角 (deg, 正北0 顺时针正)
float32                     pitch                           # H+16 俯仰角 (deg, ±90)
float32                     reserved                        # H+20 保留
float32                     heading_stdev                   # H+24 航向标准偏差
float32                     pitch_stdev                     # H+28 俯仰标准偏差
string                      rover_stn_id                    # H+32 基站ID (CHAR[4])
uint8                       num_sv_tracked                  # H+36 跟踪卫星数 (NSV2)
uint8                       num_sv_in_sol                   # H+37 解中使用的卫星数 (NSU2)
uint8                       num_sv_obs                      # H+38 截止高度角以上卫星数
uint8                       num_sv_multi                    # H+39 解中多频卫星数
uint8                       reserved1                       # H+40 保留
BestExtendedSolutionStatus  ext_sol_status                  # H+41 扩展解状态
uint8                       reserved2                       # H+42 保留
uint8                       sig_mask                        # H+43 信号掩码
