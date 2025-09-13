#include <boost/asio.hpp>
#include <string>
#include <chrono>
#include <conio.h>
#include <Windows.h>
#include <fstream>
#include <format>
#include <iostream>
#include <thread>
#include <stop_token>
#include <mutex>
#include <shared_mutex>
#include <functional>
#include <memory>

#include "head_fpslimit.hpp"
#include "ParamsServer.hpp"

using boost::asio::ip::udp;

class KeyboardRemoteControl {
private:
    static bool isKeyPressed(int vkCode) {
        return (GetAsyncKeyState(vkCode) & 0x8000) != 0;
    }

    std::stop_source sts_;

    std::string cfg_location_;

    std::string target_ip_;
    uint16_t target_port_;
	float max_linear_velocity_ = 1.500f;
    float linear_velocity_ = 0.900f;
    float angular_velocity_ = 0.300f;

public:
    double keyboard_listening_fps_ = 1000.0;
    double udp_fps_ = 240.0;

private:
    ParamsServer ps_;

    bool load_config() {
        bool tmp_res = true;
        tmp_res &= this->ps_.load_param<std::string>(this->target_ip_, "target_ip");
        tmp_res &= this->ps_.load_param<uint16_t>(this->target_port_, "target_port");
        tmp_res &= this->ps_.load_param<float>(this->max_linear_velocity_, "max_linear_velocity");
        tmp_res &= this->ps_.load_param<float>(this->linear_velocity_, "linear_velocity");
        tmp_res &= this->ps_.load_param<float>(this->angular_velocity_, "angular_velocity");
        tmp_res &= this->ps_.load_param<double>(this->keyboard_listening_fps_, "keyboard_listening_fps");
        tmp_res &= this->ps_.load_param<double>(this->udp_fps_, "udp_fps");
        return tmp_res;
    }

    void printConfigInfo() const {
        std::cout << " [ CFG ] target_link: " << this->target_ip_ << ":" << this->target_port_ << "\n";
        std::cout << " [ CFG ] max_linear_velocity: " << this->max_linear_velocity_ << "\n";
        std::cout << " [ CFG ] linear_velocity: " << this->linear_velocity_ << "\n";
        std::cout << " [ CFG ] angular_velocity: " << this->angular_velocity_ << "\n";
        std::cout << " [ CFG ] keyboard_listening_fps: " << this->keyboard_listening_fps_ << "\n";
        std::cout << " [ CFG ] udp_fps_: " << this->udp_fps_ << "\n";
    }

    struct KeyboardState {
        bool k_q, k_e;
        bool k_w, k_a, k_s, k_d;
        bool k_lshift;
        bool k_space;
        bool k_esc;
    };
    std::shared_mutex k_state_smtx_;
    KeyboardState k_state_{0};

    void kListenThreadAssist(const double fps) {
        fps_func::FPS_Limiter limiter(fps);
        while (!this->sts_.stop_requested()) {
            {
                std::unique_lock<std::shared_mutex> lck(this->k_state_smtx_);
                k_state_.k_q = isKeyPressed('Q');
                k_state_.k_e = isKeyPressed('E');
                k_state_.k_w = isKeyPressed('W');
                k_state_.k_a = isKeyPressed('A');
                k_state_.k_s = isKeyPressed('S');
                k_state_.k_d = isKeyPressed('D');
                k_state_.k_lshift = isKeyPressed(VK_LSHIFT);
                k_state_.k_space = isKeyPressed(VK_SPACE);
                k_state_.k_esc = isKeyPressed(VK_ESCAPE);
            }
            limiter.limit();
        }
    }

    struct Twist {
        float linear_x;
        float angular_z;
        long long timestamp;
    };

    bool manual_ctrl_ = false;

    boost::asio::io_context io_;
    udp::socket socket_;
    udp::endpoint remote_;

    void socketThreadAssist(const double fps) {
        Twist cmd_{0.0f, 0.0f};
        fps_func::FPS_Limiter limiter(fps);
        while (!this->sts_.stop_requested()) {
            cmd_.linear_x = 0.0f;
            cmd_.angular_z = 0.0f;
            {
                std::shared_lock<std::shared_mutex> lck(this->k_state_smtx_);
                auto [q, e, w, a, s, d, lshift, space, esc] = k_state_;
				if (esc) this->sts_.request_stop();
                if (w) cmd_.linear_x += this->linear_velocity_;
                if (s) cmd_.linear_x -= this->linear_velocity_;
                if (a) cmd_.angular_z += this->angular_velocity_;
                if (d) cmd_.angular_z -= this->angular_velocity_;
                if (lshift) cmd_.linear_x = this->max_linear_velocity_;

                if (q) this->manual_ctrl_ = false;
				if (e) this->manual_ctrl_ = true;

                long long ts = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::high_resolution_clock::now().time_since_epoch()
                ).count();

                cmd_.timestamp = this->manual_ctrl_ ? ts : -ts;
            }

            auto msg = std::format(
                "{:.3f} {:.3f} {:d}",
                cmd_.linear_x,
                cmd_.angular_z,
                cmd_.timestamp
            );
            this->socket_.send_to(boost::asio::buffer(msg), this->remote_);
            std::cout << " [ SEND ] " << msg << "\n";
            limiter.limit();
        }
    }

    std::thread klisten_thread_;
    std::thread socket_thread_;

public:
    KeyboardRemoteControl(std::string cfg_location) : cfg_location_(cfg_location), ps_(ParamsServer(cfg_location)), socket_(io_) {
        std::cout << this->cfg_location_ << std::endl;
        if(!this->load_config()) {
            this->request_stop();
            std::cerr << " [ ERROR ] Failed to load config. " << std::endl;
        } else {
            this->socket_.open(udp::v4());
            this->remote_ = udp::endpoint{ boost::asio::ip::make_address(this->target_ip_), this->target_port_ };
            this->printConfigInfo();
            std::cout << " [ INFO ] KeyboardRemoteControl Sender initialized. " << std::endl;
        }
    }
    virtual ~KeyboardRemoteControl() {
        this->sts_.request_stop();
        if(this->klisten_thread_.joinable()) this->klisten_thread_.join();
        if (this->socket_thread_.joinable()) this->socket_thread_.join();
        this->socket_.close();
        std::cout << " [ INFO ] KeyboardRemoteControl Sender end of work. " << std::endl;
    }

    void request_stop() { this->sts_.request_stop(); }
    std::stop_token get_token() { return this->sts_.get_token(); }

    void launchKeyboardListening(const double key_listen_fps, const double socket_fps) {
        if (this->sts_.stop_requested()) return;
        this->klisten_thread_ = std::thread(std::bind(&KeyboardRemoteControl::kListenThreadAssist,this,key_listen_fps));
        this->socket_thread_ = std::thread(std::bind(&KeyboardRemoteControl::socketThreadAssist,this,socket_fps));
    }
};

static void __main__exec__() {
    KeyboardRemoteControl ctrl("./cfg/cfg.txt");
    ctrl.launchKeyboardListening(ctrl.keyboard_listening_fps_, ctrl.udp_fps_);

    std::function<void(std::stop_token)> standing_by = [](std::stop_token st) ->void {
        while (!st.stop_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        return;
    };
    standing_by(ctrl.get_token());
}

static inline void async_iostream() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);
}

int main() {
    async_iostream();

    __main__exec__();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    system("pause");

    return 0;
}