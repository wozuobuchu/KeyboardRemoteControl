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

    template <typename TargetVar>
    bool load_var(TargetVar& target, std::string fileLocation) {
        std::stringstream buffer;
        std::ifstream readFile(fileLocation);
        if (!readFile.is_open()) return false;
        buffer << readFile.rdbuf();
        return static_cast<bool>(buffer >> target);
    }

    bool load_config() {
        bool tmp_res = true;
        tmp_res &= this->load_var<std::string>(this->target_ip_, this->cfg_location_+"/target_ip.txt");
        tmp_res &= this->load_var<uint16_t>(this->target_port_, this->cfg_location_ + "/target_port.txt");
        return tmp_res;
    }

    void printConfigInfo() const {
        std::cout <<" [ CFG ] target_link: "<<this->target_ip_<<":"<<this->target_port_<<std::endl;
    }

    struct KeyboardState {
        bool k_q;
        bool k_e;

        bool k_w;
        bool k_a;
        bool k_s;
        bool k_d;

        bool k_lshift;
        bool k_space;

        bool k_esc;
    };
    std::shared_mutex k_state_smtx_;
    KeyboardState k_state_;

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
    };

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
				if (w) cmd_.linear_x += 0.900f;
				if (s) cmd_.linear_x -= 0.900f;
				if (a) cmd_.angular_z += 0.300f;
				if (d) cmd_.angular_z -= 0.300f;
				if (lshift) cmd_.linear_x = 1.500f;
                //std::cout
                //    << "Q:" << q << "\n"
                //    << "E:" << e << "\n"
                //    << "W:" << w << "\n"
                //    << "A:" << a << "\n"
                //    << "S:" << s << "\n"
                //    << "D:" << d << "\n"
                //    << "L_SHIFT:" << lshift << "\n"
                //    << "SPACE:" << space << "\n"
                //    << "ESC:" << esc << "\n"
                //    << std::endl;
            }
            auto msg = std::format("{:.3f} {:.3f}", cmd_.linear_x, cmd_.angular_z);
            this->socket_.send_to(boost::asio::buffer(msg), this->remote_);
            std::cout << " [ SEND ] " << msg << std::endl;
            limiter.limit();
        }
    }

    std::thread klisten_thread_;
    std::thread socket_thread_;

public:
    KeyboardRemoteControl(std::string cfg_location) : cfg_location_(cfg_location), socket_(io_) {
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
        this->socket_.close();
        if(this->klisten_thread_.joinable()) this->klisten_thread_.join();
	if (this->socket_thread_.joinable()) this->socket_thread_.join();
        std::cout << " [ INFO ] KeyboardRemoteControl Sender end of work. " << std::endl;
    }

    void request_stop() { this->sts_.request_stop(); }
    std::stop_token get_token() { return this->sts_.get_token(); }

    void launchKeyboardListening(const double key_listen_fps, const double socket_fps) {
        this->klisten_thread_ = std::thread(std::bind(&KeyboardRemoteControl::kListenThreadAssist,this,key_listen_fps));
        this->socket_thread_ = std::thread(std::bind(&KeyboardRemoteControl::socketThreadAssist,this,socket_fps));
    }
};

static void __main__exec__() {
    KeyboardRemoteControl ctrl("./cfg");
    ctrl.launchKeyboardListening(1000.0, 120.0);

    std::function<void(std::stop_token)> standing_by = [](std::stop_token st) ->void {
        while (!st.stop_requested()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return;
    };
    standing_by(ctrl.get_token());
}

int main() {
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);
    std::cout.tie(nullptr);

    __main__exec__();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    system("pause");

    return 0;
}
