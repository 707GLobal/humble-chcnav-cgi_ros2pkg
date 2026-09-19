#pragma once

#include "msg_interfaces/msg/bestpos.hpp"
#include "msg_interfaces/msg/heading.hpp"
#include "msg_interfaces/msg/arc_header.hpp"
#include "msg_interfaces/msg/solution_status.hpp"
#include "msg_interfaces/msg/position_or_velocity_type.hpp"
#include "msg_interfaces/msg/best_extended_solution_status.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdint>
#include <cstring>
#include <vector>

#define NOVATEL_HEADER_LEN     28
#define NOVATEL_MSG_ID_BESTPOS 42
#define NOVATEL_MSG_ID_HEADING 971
#define NOVATEL_BESTPOS_LEN    (NOVATEL_HEADER_LEN + 72 + 4)
#define NOVATEL_HEADING_LEN    (NOVATEL_HEADER_LEN + 44 + 4)

/* 小端读取工具 */
static inline uint16_t nv_le_u16(const std::vector<int8_t> &d, size_t off)
{
    return (uint16_t)(uint8_t)d[off] | ((uint16_t)(uint8_t)d[off + 1] << 8);
}

static inline uint32_t nv_le_u32(const std::vector<int8_t> &d, size_t off)
{
    return (uint32_t)(uint8_t)d[off] |
           ((uint32_t)(uint8_t)d[off + 1] << 8) |
           ((uint32_t)(uint8_t)d[off + 2] << 16) |
           ((uint32_t)(uint8_t)d[off + 3] << 24);
}

static inline float nv_le_f32(const std::vector<int8_t> &d, size_t off)
{
    float v; memcpy(&v, &d[off], 4); return v;
}

static inline double nv_le_f64(const std::vector<int8_t> &d, size_t off)
{
    double v; memcpy(&v, &d[off], 8); return v;
}

static inline void nv_fill_arc_header(msg_interfaces::msg::ArcHeader &hdr,
                                      const std::vector<int8_t> &d)
{
    hdr.header_len      = (uint8_t)d[3];
    hdr.msg_id          = nv_le_u16(d, 4);
    hdr.msg_type        = (uint8_t)d[6];
    hdr.port_addr       = (uint8_t)d[7];
    hdr.msg_length      = nv_le_u16(d, 8);
    hdr.sequence        = nv_le_u16(d, 10);
    hdr.idle_time       = (uint8_t)d[12];
    hdr.time_status     = (uint8_t)d[13];
    hdr.gps_week        = nv_le_u16(d, 14);
    hdr.gps_ms          = nv_le_u32(d, 16);
    hdr.receiver_status = nv_le_u32(d, 20);
    hdr.reserved        = nv_le_u16(d, 24);
    hdr.sw_version      = nv_le_u16(d, 26);
}

static inline rclcpp::Time nv_gps_to_ros_time(uint16_t week, uint32_t ms, int leaps)
{
    double gps_sec = week * 7.0 * 24.0 * 3600.0 + ms / 1000.0;
    return rclcpp::Time((gps_sec + 315964800.0 - leaps) * 1e9);
}

/* 解码并发布 BESTPOSB 帧 */
static inline void nv_decode_bestpos(
    const std::vector<int8_t> &d,
    const rclcpp::Time &recv_stamp,
    int leaps,
    rclcpp::Publisher<msg_interfaces::msg::BESTPOS>::SharedPtr &pub,
    const rclcpp::Logger &logger)
{
    if (d.size() != NOVATEL_BESTPOS_LEN)
    {
        RCLCPP_WARN(logger, "BESTPOSB length error[%zu]", d.size());
        return;
    }
    const size_t H = NOVATEL_HEADER_LEN;
    msg_interfaces::msg::BESTPOS p;

    nv_fill_arc_header(p.arc_header, d);
    p.header.stamp = nv_gps_to_ros_time(p.arc_header.gps_week, p.arc_header.gps_ms, leaps);

    p.sol_status.value  = (uint8_t)nv_le_u32(d, H + 0);
    p.pos_type.value    = (uint8_t)nv_le_u32(d, H + 4);
    p.lat               = nv_le_f64(d, H + 8);
    p.lon               = nv_le_f64(d, H + 16);
    p.hgt               = nv_le_f64(d, H + 24);
    p.undulation        = nv_le_f32(d, H + 32);
    p.datum_id          = nv_le_u32(d, H + 36);
    p.lat_stdev         = nv_le_f32(d, H + 40);
    p.lon_stdev         = nv_le_f32(d, H + 44);
    p.hgt_stdev         = nv_le_f32(d, H + 48);
    for (int i = 0; i < 4; i++)
        p.stn_id[i] = d[H + 52 + i];
    p.diff_age          = nv_le_f32(d, H + 56);
    p.sol_age           = nv_le_f32(d, H + 60);
    p.num_svs           = (uint8_t)d[H + 64];
    p.num_sol_svs       = (uint8_t)d[H + 65];
    p.num_sol_l1_svs    = (uint8_t)d[H + 66];
    p.num_sol_multi_svs = (uint8_t)d[H + 67];
    p.reserved          = (uint8_t)d[H + 68];
    p.ext_sol_stat.value= (uint8_t)d[H + 69];
    p.reserved1         = (uint8_t)d[H + 70];
    p.sig_mask          = (uint8_t)d[H + 71];

    pub->publish(p);
    RCLCPP_DEBUG(logger, "BESTPOSB published: sol_status=%d pos_type=%d lat=%.9f lon=%.9f",
                 p.sol_status.value, p.pos_type.value, p.lat, p.lon);
}

/* 解码并发布 HEADINGB 帧 */
static inline void nv_decode_heading(
    const std::vector<int8_t> &d,
    const rclcpp::Time &recv_stamp,
    int leaps,
    rclcpp::Publisher<msg_interfaces::msg::HEADING>::SharedPtr &pub,
    const rclcpp::Logger &logger)
{
    if (d.size() != NOVATEL_HEADING_LEN)
    {
        RCLCPP_WARN(logger, "HEADINGB length error[%zu]", d.size());
        return;
    }
    const size_t H = NOVATEL_HEADER_LEN;
    msg_interfaces::msg::HEADING h;

    nv_fill_arc_header(h.arc_header, d);
    h.header.stamp = nv_gps_to_ros_time(h.arc_header.gps_week, h.arc_header.gps_ms, leaps);

    h.sol_status.value  = (uint8_t)nv_le_u32(d, H + 0);
    h.pos_type.value    = (uint8_t)nv_le_u32(d, H + 4);
    h.length            = nv_le_f32(d, H + 8);
    h.heading           = nv_le_f32(d, H + 12);
    h.pitch             = nv_le_f32(d, H + 16);
    h.reserved          = nv_le_f32(d, H + 20);
    h.heading_stdev     = nv_le_f32(d, H + 24);
    h.pitch_stdev       = nv_le_f32(d, H + 28);
    h.rover_stn_id.assign(reinterpret_cast<const char *>(&d[H + 32]),
                          strnlen(reinterpret_cast<const char *>(&d[H + 32]), 4));
    h.num_sv_tracked    = (uint8_t)d[H + 36];
    h.num_sv_in_sol     = (uint8_t)d[H + 37];
    h.num_sv_obs        = (uint8_t)d[H + 38];
    h.num_sv_multi      = (uint8_t)d[H + 39];
    h.reserved1         = (uint8_t)d[H + 40];
    h.ext_sol_status.value = (uint8_t)d[H + 41];
    h.reserved2         = (uint8_t)d[H + 42];
    h.sig_mask          = (uint8_t)d[H + 43];

    pub->publish(h);
    RCLCPP_DEBUG(logger, "HEADINGB published: sol_status=%d pos_type=%d heading=%.3f pitch=%.3f",
                 h.sol_status.value, h.pos_type.value, h.heading, h.pitch);
}
