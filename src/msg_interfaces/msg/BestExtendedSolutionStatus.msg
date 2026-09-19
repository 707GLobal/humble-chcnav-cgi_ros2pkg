# 附录4：扩展解状态 (Extended Solution Status)，位掩码，1 字节
uint8 value

uint8 NONE = 0
uint8 LEAP_SECOND_KNOWN = 0x01
uint8 LEAP_SECOND_PENDING = 0x02
uint8 RTCALIBRATED = 0x10
uint8 RTCALIBRATION_IN_PROGRESS = 0x40
uint8 RTCALIBRATION_UNKNOWN = 0x80
