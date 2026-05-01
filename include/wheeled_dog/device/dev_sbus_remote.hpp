#ifndef __DEV_SBUS_REMOTE_HPP
#define __DEV_SBUS_REMOTE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <string>

class SbusRemoteController : public rclcpp::Node
{
public:
    SbusRemoteController();
    ~SbusRemoteController() override;

private:
    static constexpr std::size_t SBUS_FRAME_SIZE = 25;
    static constexpr std::size_t SBUS_CHANNEL_COUNT = 16;

    struct SbusFrame
    {
        std::array<uint16_t, SBUS_CHANNEL_COUNT> channels{};
        bool frame_lost = false;
        bool failsafe = false;
    };

    struct RemoteChannels
    {
        std::array<int, 4> sticks{};
        int switch_5 = 0;
        int switch_6 = 0;
        int switch_7 = 0;
        int switch_8 = 0;
        int knob_9 = 0;
        int knob_10 = 0;
        double linear_x = 0.0;
        double angular_z = 0.0;
    };

    void load_parameters();
    void open_serial();
    void close_serial();
    bool configure_serial();
    void poll_serial();
    void check_timeout();
    void publish_current_command();
    bool parse_byte(uint8_t byte, SbusFrame & frame);
    SbusFrame decode_frame(const std::array<uint8_t, SBUS_FRAME_SIZE> & frame_bytes) const;
    bool is_valid_frame(const std::array<uint8_t, SBUS_FRAME_SIZE> & frame_bytes) const;
    RemoteChannels process_channels(const SbusFrame & frame) const;
    int stick_to_500(uint16_t raw) const;
    double stick_to_unit(uint16_t raw) const;
    int three_position_switch(uint16_t raw) const;
    int two_position_switch(uint16_t raw) const;
    int knob_to_1000(uint16_t raw) const;
    void set_current_command(double linear_x, double angular_z);
    void publish_twist(double linear_x, double angular_z);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr poll_timer_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::string port_;
    std::string topic_name_;
    int baud_rate_ = 100000;
    int poll_period_ms_ = 5;
    int timeout_ms_ = 300;
    int raw_min_ = 172;
    int raw_mid_ = 992;
    int raw_max_ = 1811;
    int deadzone_ = 50;
    bool invert_angular_z_ = true;
    bool debug_output_ = false;

    int fd_ = -1;
    std::array<uint8_t, SBUS_FRAME_SIZE> rx_frame_{};
    std::size_t rx_index_ = 0;
    double latest_linear_x_ = 0.0;
    double latest_angular_z_ = 0.0;
    rclcpp::Time last_frame_time_;
};

#endif
