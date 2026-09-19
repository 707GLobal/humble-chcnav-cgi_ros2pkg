# 附录2：解状态 (Solution Status)，NovAtel OEM7 枚举
# 注意：协议中该字段为 4 字节，取值见下方常量；请按 CHC 附录核对
uint8 value

uint8 NONE = 0
uint8 FIXEDPOS = 1
uint8 FIXEDHEIGHT = 2
uint8 FLOATCONV = 3
uint8 WIDELANE = 4
uint8 NARROWLANE = 5
uint8 DGPS = 6
uint8 OMNI = 7
uint8 PSRDIFF = 8
uint8 PPP_CONVERGING = 9
uint8 PPP = 10
uint8 INS_COARSE = 11
uint8 INS_PSRSP = 12
uint8 INS_PROPAGATED = 13
uint8 INS_FLOAT = 14
uint8 INS_INT = 15
uint8 INS_NARROW_INT = 16
uint8 INS_PSRSP_NEW = 17
uint8 INS_RAW = 18
uint8 INS_PPP_CONVERGING = 19
uint8 INS_PPP = 20
uint8 INS_RTKDIRECT = 21
uint8 INS_SBAS = 22
uint8 INS_PPP_SR = 23
