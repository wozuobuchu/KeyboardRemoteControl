#include <boost/asio.hpp>
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <functional>
#include <utility>
#include <stop_token>
#include <latch>
#include <condition_variable>

using boost::asio::ip::udp;
using namespace std::chrono_literals;

class UdpToCmdVel : public rclcpp::Node {
private:
    std::stop_source sts_;

    int tsign(long long TimeStamp) { return ( (TimeStamp>0) - (TimeStamp<0) ); }

    std::shared_mutex manual_ctrl_smtx_;
    bool manual_ctrl_ = false;

    boost::asio::io_context io_;
    udp::socket socket_;
    udp::endpoint remote_endpoint_;
    std::array<char, 64> recv_buffer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
    long long timestamp_last_ = 0;
    void start_receive() {
        socket_.async_receive_from(
            boost::asio::buffer(recv_buffer_), remote_endpoint_,
            [this](boost::system::error_code ec, std::size_t bytes_recvd) ->void {
                if (!ec && bytes_recvd > 0) {
                    std::string data(recv_buffer_.data(), bytes_recvd);
                    float lin=0.0f, ang=0.0f;
                    long long timestamp=1;
                    if (std::sscanf(data.c_str(), "%f%f%lld", &lin, &ang, &timestamp) == 3) {
                        if (timestamp>0) {
                            geometry_msgs::msg::Twist msg;
                            msg.linear.x = lin;
                            msg.angular.z = ang;
                            pub_->publish(msg);
                            if(tsign(timestamp)!=tsign(timestamp_last_)) {
                                std::unique_lock<std::shared_mutex> lck(this->manual_ctrl_smtx_);
                                this->manual_ctrl_ = true;
                            }
                        } else if ((timestamp<0) && (tsign(timestamp)!=tsign(timestamp_last_))) {
                            std::unique_lock<std::shared_mutex> lck(this->manual_ctrl_smtx_);
                            this->manual_ctrl_ = false;
                        }
                        timestamp_last_ = timestamp;
                    } else {
                        RCLCPP_WARN(this->get_logger(),"Failed to parse: '%s'", data.c_str());
                    }
                }
                this->start_receive();
            }
        );
    }

    void cmd_relay_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        bool f;
        {
            std::shared_lock<std::shared_mutex> lck(this->manual_ctrl_smtx_);
            f = this->manual_ctrl_;
        }

        if(f==false) {
            geometry_msgs::msg::Twist twist;
            twist.linear.x = msg->linear.x;
            twist.angular.z = msg->angular.z;
            this->pub_->publish(twist);
        }
    }

    std::thread listener_thread_;

public:
    UdpToCmdVel(int port) : Node("udp_to_cmdvel"), io_(), socket_(io_, udp::endpoint(udp::v4(), port)) {
        std::stringstream ss;
        ss<<port;
        std::string port_str;
        ss>>port_str;
        this->pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        this->sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_relay", 10,
            std::bind(&UdpToCmdVel::cmd_relay_callback,this,std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), (std::string("")+"UDP listener bound to port"+port_str).c_str());
        this->listener_thread_ = std::thread([this]() ->void { io_.run(); });
        this->start_receive();
    }

    ~UdpToCmdVel() {
        this->io_.stop();
        if (listener_thread_.joinable()) listener_thread_.join();
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<UdpToCmdVel>(5005));

    rclcpp::shutdown();
    return 0;
}