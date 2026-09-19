# 附录3：位置类型/速度类型 (Position Type / Velocity Type)，NovAtel OEM7 枚举
# 注意：协议中该字段为 4 字节，取值见下方常量；请按 CHC 附录核对
uint8 value

uint8 NONE = 0
uint8 FIXEDPOS = 1
uint8 FIXEDHEIGHT = 2
uint8 FLOATCONV = 4
uint8 WIDELANE = 5
uint8 NARROWLANE = 6
uint8 DGPS = 7
uint8 OMNI = 8
uint8 PSRDIFF = 9
uint8 WAAS = 10
uint8 PROPAGATED = 11
uint8 MANUAL = 12
uint8 PPP_CONVERGING = 17
uint8 PPP = 18
uint8 INS_COARSE = 32
uint8 INS_PSRSP = 33
uint8 INS_PROPAGATED = 34
uint8 INS_FLOAT = 35
uint8 INS_INT = 36
uint8 INS_NARROW_INT = 37
uint8 INS_PSRSP_NEW = 38
uint8 INS_RAW = 40
uint8 INS_PPP_CONVERGING = 41
uint8 INS_PPP = 42
uint8 INS_RTKDIRECT = 50
uint8 INS_SBAS = 51
uint8 INS_PPP_SR = 52
