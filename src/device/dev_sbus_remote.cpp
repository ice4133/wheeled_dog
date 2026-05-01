#include "wheeled_dog/device/dev_sbus_remote.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <sys/ioctl.h>
#include <unistd.h>

#include <asm/termbits.h>

namespace
{
constexpr uint8_t SBUS_HEADER = 0x0F;
constexpr uint8_t SBUS_FLAG_FRAME_LOST = 0x04;
constexpr uint8_t SBUS_FLAG_FAILSAFE = 0x08;
constexpr double STICK_OUTPUT_LIMIT = 500.0;
constexpr double KNOB_OUTPUT_LIMIT = 1000.0;
constexpr int CMD_VEL_PUBLISH_PERIOD_MS = 20;
}

SbusRemoteController::SbusRemoteController() : Node("sbus_remote_controller_node")
{
    load_parameters();

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(topic_name_, 10);
    last_frame_time_ = this->now();

    open_serial();

    poll_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(poll_period_ms_),
        std::bind(&SbusRemoteController::poll_serial, this));
    timeout_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timeout_ms_),
        std::bind(&SbusRemoteController::check_timeout, this));
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(CMD_VEL_PUBLISH_PERIOD_MS),
        std::bind(&SbusRemoteController::publish_current_command, this));

    RCLCPP_INFO(
        this->get_logger(),
        "SBUS遥控器节点已启动: port=%s baud=%d topic=%s publish_rate=50Hz",
        port_.c_str(),
        baud_rate_,
        topic_name_.c_str());
}

SbusRemoteController::~SbusRemoteController()
{
    publish_twist(0.0, 0.0);
    close_serial();
    RCLCPP_INFO(this->get_logger(), "SBUS遥控器节点已关闭，已发送停止指令。");
}

void SbusRemoteController::load_parameters()
{
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyUSB5");
    topic_name_ = this->declare_parameter<std::string>("topic", "cmd_vel");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 100000);
    poll_period_ms_ = this->declare_parameter<int>("poll_period_ms", 5);
    timeout_ms_ = this->declare_parameter<int>("timeout_ms", 300);
    raw_min_ = this->declare_parameter<int>("raw_min", 172);
    raw_mid_ = this->declare_parameter<int>("raw_mid", 992);
    raw_max_ = this->declare_parameter<int>("raw_max", 1811);
    deadzone_ = this->declare_parameter<int>("deadzone", 50);
    invert_angular_z_ = this->declare_parameter<bool>("invert_angular_z", true);
    debug_output_ = this->declare_parameter<bool>("debug_output", false);

    poll_period_ms_ = std::max(1, poll_period_ms_);
    timeout_ms_ = std::max(20, timeout_ms_);
    raw_mid_ = std::clamp(raw_mid_, raw_min_ + 1, raw_max_ - 1);
    deadzone_ = std::clamp(deadzone_, 0, static_cast<int>(STICK_OUTPUT_LIMIT));
}

void SbusRemoteController::open_serial()
{
    if (fd_ >= 0) {
        return;
    }

    fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "无法打开SBUS串口 %s: %s",
            port_.c_str(),
            std::strerror(errno));
        return;
    }

    if (!configure_serial()) {
        close_serial();
        return;
    }

    RCLCPP_INFO(this->get_logger(), "已打开SBUS串口 %s", port_.c_str());
}

void SbusRemoteController::close_serial()
{
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
    rx_index_ = 0;
}

bool SbusRemoteController::configure_serial()
{
    struct termios2 options {};
    if (ioctl(fd_, TCGETS2, &options) != 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "读取串口配置失败 %s: %s",
            port_.c_str(),
            std::strerror(errno));
        return false;
    }

    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cflag &= ~CBAUD;
    options.c_cflag |= BOTHER;
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_cflag |= CSTOPB;
    options.c_ispeed = baud_rate_;
    options.c_ospeed = baud_rate_;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (ioctl(fd_, TCSETS2, &options) != 0) {
        RCLCPP_ERROR(
            this->get_logger(),
            "配置串口失败 %s: %s",
            port_.c_str(),
            std::strerror(errno));
        return false;
    }

    return true;
}

void SbusRemoteController::poll_serial()
{
    if (fd_ < 0) {
        open_serial();
        return;
    }

    uint8_t buffer[256];
    while (true) {
        const ssize_t bytes_read = read(fd_, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            for (ssize_t i = 0; i < bytes_read; ++i) {
                SbusFrame frame;
                if (parse_byte(buffer[i], frame)) {
                    last_frame_time_ = this->now();

                    if (frame.failsafe || frame.frame_lost) {
                        set_current_command(0.0, 0.0);
                        RCLCPP_WARN_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            1000,
                            "SBUS帧丢失或进入failsafe，已发送停止指令。");
                        continue;
                    }

                    const RemoteChannels remote = process_channels(frame);
                    set_current_command(remote.linear_x, remote.angular_z);

                    if (debug_output_) {
                        RCLCPP_INFO_THROTTLE(
                            this->get_logger(),
                            *this->get_clock(),
                            500,
                            "CH1=%d CH3=%d linear.x=%.3f angular.z=%.3f SW5=%d SW6=%d SW7=%d SW8=%d K9=%d K10=%d",
                            remote.sticks[0],
                            remote.sticks[2],
                            remote.linear_x,
                            remote.angular_z,
                            remote.switch_5,
                            remote.switch_6,
                            remote.switch_7,
                            remote.switch_8,
                            remote.knob_9,
                            remote.knob_10);
                    }
                }
            }
            continue;
        }

        if (bytes_read == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }

        RCLCPP_WARN(
            this->get_logger(),
            "读取SBUS串口失败 %s: %s",
            port_.c_str(),
            std::strerror(errno));
        close_serial();
        set_current_command(0.0, 0.0);
        break;
    }
}

void SbusRemoteController::check_timeout()
{
    const auto elapsed = this->now() - last_frame_time_;
    if (elapsed.nanoseconds() > static_cast<int64_t>(timeout_ms_) * 1000000LL) {
        set_current_command(0.0, 0.0);
        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "超过%d ms未收到有效SBUS数据，已发送停止指令。",
            timeout_ms_);
    }
}

void SbusRemoteController::publish_current_command()
{
    publish_twist(latest_linear_x_, latest_angular_z_);
}

bool SbusRemoteController::parse_byte(uint8_t byte, SbusFrame & frame)
{
    if (rx_index_ == 0) {
        if (byte != SBUS_HEADER) {
            return false;
        }
        rx_frame_[rx_index_++] = byte;
        return false;
    }

    rx_frame_[rx_index_++] = byte;
    if (rx_index_ < SBUS_FRAME_SIZE) {
        return false;
    }

    rx_index_ = 0;
    if (!is_valid_frame(rx_frame_)) {
        return false;
    }

    frame = decode_frame(rx_frame_);
    return true;
}

SbusRemoteController::SbusFrame SbusRemoteController::decode_frame(
    const std::array<uint8_t, SBUS_FRAME_SIZE> & frame_bytes) const
{
    SbusFrame frame;
    const auto & b = frame_bytes;

    frame.channels[0] = static_cast<uint16_t>((b[1] | b[2] << 8) & 0x07FF);
    frame.channels[1] = static_cast<uint16_t>((b[2] >> 3 | b[3] << 5) & 0x07FF);
    frame.channels[2] = static_cast<uint16_t>((b[3] >> 6 | b[4] << 2 | b[5] << 10) & 0x07FF);
    frame.channels[3] = static_cast<uint16_t>((b[5] >> 1 | b[6] << 7) & 0x07FF);
    frame.channels[4] = static_cast<uint16_t>((b[6] >> 4 | b[7] << 4) & 0x07FF);
    frame.channels[5] = static_cast<uint16_t>((b[7] >> 7 | b[8] << 1 | b[9] << 9) & 0x07FF);
    frame.channels[6] = static_cast<uint16_t>((b[9] >> 2 | b[10] << 6) & 0x07FF);
    frame.channels[7] = static_cast<uint16_t>((b[10] >> 5 | b[11] << 3) & 0x07FF);
    frame.channels[8] = static_cast<uint16_t>((b[12] | b[13] << 8) & 0x07FF);
    frame.channels[9] = static_cast<uint16_t>((b[13] >> 3 | b[14] << 5) & 0x07FF);
    frame.channels[10] = static_cast<uint16_t>((b[14] >> 6 | b[15] << 2 | b[16] << 10) & 0x07FF);
    frame.channels[11] = static_cast<uint16_t>((b[16] >> 1 | b[17] << 7) & 0x07FF);
    frame.channels[12] = static_cast<uint16_t>((b[17] >> 4 | b[18] << 4) & 0x07FF);
    frame.channels[13] = static_cast<uint16_t>((b[18] >> 7 | b[19] << 1 | b[20] << 9) & 0x07FF);
    frame.channels[14] = static_cast<uint16_t>((b[20] >> 2 | b[21] << 6) & 0x07FF);
    frame.channels[15] = static_cast<uint16_t>((b[21] >> 5 | b[22] << 3) & 0x07FF);

    frame.frame_lost = (b[23] & SBUS_FLAG_FRAME_LOST) != 0;
    frame.failsafe = (b[23] & SBUS_FLAG_FAILSAFE) != 0;
    return frame;
}

bool SbusRemoteController::is_valid_frame(const std::array<uint8_t, SBUS_FRAME_SIZE> & frame_bytes) const
{
    return frame_bytes[0] == SBUS_HEADER;
}

SbusRemoteController::RemoteChannels SbusRemoteController::process_channels(const SbusFrame & frame) const
{
    RemoteChannels remote;
    for (std::size_t i = 0; i < remote.sticks.size(); ++i) {
        remote.sticks[i] = stick_to_500(frame.channels[i]);
    }

    remote.switch_5 = three_position_switch(frame.channels[4]);
    remote.switch_6 = two_position_switch(frame.channels[5]);
    remote.switch_7 = two_position_switch(frame.channels[6]);
    remote.switch_8 = three_position_switch(frame.channels[7]);
    remote.knob_9 = knob_to_1000(frame.channels[8]);
    remote.knob_10 = knob_to_1000(frame.channels[9]);

    remote.linear_x = static_cast<double>(remote.sticks[2]) / STICK_OUTPUT_LIMIT;
    remote.angular_z = static_cast<double>(remote.sticks[0]) / STICK_OUTPUT_LIMIT;
    if (invert_angular_z_) {
        remote.angular_z = -remote.angular_z;
    }

    return remote;
}

int SbusRemoteController::stick_to_500(uint16_t raw) const
{
    const double unit = stick_to_unit(raw);
    int value = static_cast<int>(std::lround(unit * STICK_OUTPUT_LIMIT));
    if (std::abs(value) <= deadzone_) {
        return 0;
    }
    return std::clamp(value, -static_cast<int>(STICK_OUTPUT_LIMIT), static_cast<int>(STICK_OUTPUT_LIMIT));
}

double SbusRemoteController::stick_to_unit(uint16_t raw) const
{
    double value = 0.0;
    if (raw >= raw_mid_) {
        value = static_cast<double>(raw - raw_mid_) / static_cast<double>(raw_max_ - raw_mid_);
    } else {
        value = static_cast<double>(raw) - static_cast<double>(raw_mid_);
        value /= static_cast<double>(raw_mid_ - raw_min_);
    }
    return std::clamp(value, -1.0, 1.0);
}

int SbusRemoteController::three_position_switch(uint16_t raw) const
{
    const int lower_threshold = raw_min_ + (raw_max_ - raw_min_) / 3;
    const int upper_threshold = raw_min_ + 2 * (raw_max_ - raw_min_) / 3;

    if (raw < lower_threshold) {
        return 0;
    }
    if (raw < upper_threshold) {
        return 1;
    }
    return 2;
}

int SbusRemoteController::two_position_switch(uint16_t raw) const
{
    return raw < raw_mid_ ? 0 : 1;
}

int SbusRemoteController::knob_to_1000(uint16_t raw) const
{
    const double ratio = (static_cast<double>(raw) - static_cast<double>(raw_min_)) /
                         static_cast<double>(raw_max_ - raw_min_);
    const double clamped = std::clamp(ratio, 0.0, 1.0);
    return static_cast<int>(std::lround(clamped * KNOB_OUTPUT_LIMIT));
}

void SbusRemoteController::set_current_command(double linear_x, double angular_z)
{
    latest_linear_x_ = std::clamp(linear_x, -1.0, 1.0);
    latest_angular_z_ = std::clamp(angular_z, -1.0, 1.0);
}

void SbusRemoteController::publish_twist(double linear_x, double angular_z)
{
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;
    publisher_->publish(twist_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SbusRemoteController>());
    rclcpp::shutdown();
    return 0;
}
