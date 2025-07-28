#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "dsr_msgs2/srv/move_joint.hpp"
#include "dsr_msgs2/srv/move_line.hpp"
#include "dsr_msgs2/srv/move_jointx.hpp"
#include <thread>
#include <chrono>
#include <iostream>
#include <memory>
#define DR_BASE 0
#define DR_TOOL 1
#define DR_WORLD 2

// Các chế độ chuyển động (mode)
#define DR_MV_MOD_ABS 0 // chuyển động tuyệt đối
#define DR_MV_MOD_REL 1 // chuyển động tương đối
using namespace std::chrono_literals;
class ControllerNode : public rclcpp::Node
{
public:
    ControllerNode()
        : Node("controller_node")
    {
        client_movej_ = this->create_client<dsr_msgs2::srv::MoveJoint>("/dsr01/motion/move_joint");
        client_movel_ = this->create_client<dsr_msgs2::srv::MoveLine>("/dsr01/motion/move_line");
        client_movej_x = this->create_client<dsr_msgs2::srv::MoveJointx>("/dsr01/motion/move_jointx");
        while (!client_movej_->wait_for_service(std::chrono::seconds(1)) ||
               !client_movel_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for service...");
        }

        // running = true;

        // Luồng 1: Nhập script từ bàn phím
        std::thread input_thread([this]()
                                 {
            int user_input;
            while (rclcpp::ok()) {
                
                if (this->state[0] == 1 || this->state[1] == 1 || this->state[2] == 1)
                {
                    RCLCPP_INFO(this->get_logger(), "Robot đang bận, vui lòng đợi.");
                    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // tránh quá tải CPU
                    continue;
                }
                else{
                    int user_input;
                    std::cout << "Nhập script [0: reset, 1: A, 2: B, 3: C]: ";
                    std::cin >> user_input;
                    this->script = user_input;
                    if(this->script > -1 || this->script < 4)
                    {
                        this->state[this->script-1] = 1; 
                    }
                    
                }
                
                
            
            } });
        // Thread 2: chạy script
        std::thread run_thread([this]()
                               { this->chose_script(); });

        input_thread.detach();
        run_thread.detach();
    }
    void chose_script()
    {
        while (rclcpp::ok())
        {
            if (script < -2 || script > 3)
            {
                RCLCPP_ERROR(this->get_logger(), "Script invalid: %d", script);
                rclcpp::sleep_for(std::chrono::milliseconds(100)); // tránh CPU 100%
                break;
            }
            else
            {
                if (script == 1)
                {
                    // reset();
                    // std::cout<<this->state[0] << " " << this->state[1] << " " << this->state[2] << std::endl;

                    A();
                }
                else if (script == 2)
                {
                    // reset();
                    // RCLCPP_INFO(this->get_logger(), "Chạy script B");
                    B();
                }
                else if (script == 3)
                {
                    // reset();
                    // RCLCPP_INFO(this->get_logger(), "Chạy script C");
                    C();
                }
            }
        }
    }
    int script = -1;
    int state[3] = {0, 0, 0};

private:
    std::mutex mtx;
    rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedPtr client_movej_;
    rclcpp::Client<dsr_msgs2::srv::MoveLine>::SharedPtr client_movel_;
    rclcpp::Client<dsr_msgs2::srv::MoveJointx>::SharedPtr client_movej_x;
    void A()
    {
        if (this->state[0] == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Chạy script A");
            reset();
            auto start = std::chrono::steady_clock::now();
            while (rclcpp::ok() && std::chrono::steady_clock::now() - start < std::chrono::seconds(15))
            {
                move_joint({0.0, 0.0, 90.0, 0.0, 90.0, 0.0});
                move_joint({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
            }
            this->state[0] = 0;
        }
    }

    void B()
    {
        if (this->state[1] == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Chạy script B");
            move_joint({0, 0, 0, 0, 0, 0});
            move_joint({0.0, 0.0, 90.0, 0.0, 90.0, 0.0});

            auto start = std::chrono::steady_clock::now();

            while (rclcpp::ok() && std::chrono::steady_clock::now() - start < std::chrono::seconds(15))
            {
                move_line({503, -353, 290.0, 0.0, 180.0, 0.0});
                move_line({503.0, -353.0, 190.0, 0.0, 180.0, 0.0});
                move_line({503, -353, 290.0, 0.0, 180.0, 0.0});
                move_line({503, 210, 400.0, 0.0, 180.0, 0.0});
                move_line({503, 550, 420, 0, 180, 0});
                move_joint({0.0, 0.0, 90.0, 0.0, 90.0, 0.0});

                rclcpp::sleep_for(std::chrono::milliseconds(100)); // tránh quá tải CPU
            }
        }
        /*state[1] = 1;
         */
        state[1] = 0;
    }

    void C()
    {
        // Implement the sequence of movements here
        // Similar to the move_to_pose_sequence in MoveRobotNode
        if (this->state[2] == 1)
        {
            std::array<double, 2> velx = {30, 20};
            std::array<double, 2> accx = {60, 40};

            std::array<double, 6> JReady = {0, -20, 110, 0, 60, 0};
            std::array<double, 6> TCP_POS = {0, 0, 0, 0, 0, 0};
            std::array<double, 6> J00 = {-180, 0, -145, 0, -35, 0};

            std::array<double, 6> J01r = {-180.0, 71.4, -145.0, 0.0, -9.7, 0.0};
            std::array<double, 6> J02r = {-180.0, 67.7, -144.0, 0.0, 76.3, 0.0};
            std::array<double, 6> J03r = {-180.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            std::array<double, 6> J04r = {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            std::array<double, 6> J04r1 = {-90.0, 30.0, -60.0, 0.0, 30.0, 0.0};
            std::array<double, 6> J04r2 = {-90.0, -45.0, 90.0, 0.0, -45.0, 0.0};
            std::array<double, 6> J04r3 = {-90.0, 60.0, -120.0, 0.0, 60.0, 0.0};
            std::array<double, 6> J04r4 = {-90.0, 0.0, 0.0, 0.0, 0.0, 0.0};

            std::array<double, 6> J05r = {-144.0, -4.0, -84.8, -90.9, 54.0, -1.1};
            std::array<double, 6> J07r = {-152.4, 12.4, -78.6, 18.7, -68.3, -37.7};
            std::array<double, 6> J08r = {-90.0, 30.0, -120.0, -90.0, -90.0, 0.0};

            std::array<double, 6> JEnd = {0.0, -12.6, 101.1, 0.0, 91.5, 0.0};

            std::array<double, 6> dREL1 = {0, 0, 350, 0, 0, 0};
            std::array<double, 6> dREL2 = {0, 0, -350, 0, 0, 0};

            std::array<double, 2> vel_spi = {400, 400};
            std::array<double, 2> acc_spi = {150, 150};

            std::array<double, 6> J1 = {81.2, 20.8, 127.8, 162.5, 56.1, -37.1};
            std::array<double, 6> X0 = {-88.7, 799.0, 182.3, 95.7, 93.7, 133.9};
            std::array<double, 6> X1 = {304.2, 871.8, 141.5, 99.5, 84.9, 133.4};
            std::array<double, 6> X2 = {437.1, 876.9, 362.1, 99.6, 84.0, 132.1};
            std::array<double, 6> X3 = {-57.9, 782.4, 478.4, 99.6, 84.0, 132.1};

            std::array<double, 6> amp = {0, 0, 0, 30, 30, 0};
            std::array<double, 6> period = {0, 0, 0, 3, 6, 0};

            std::array<double, 6> x01 = {423.6, 334.5, 651.2, 84.7, -180.0, 84.7};
            std::array<double, 6> x02 = {423.6, 34.5, 951.2, 68.2, -180.0, 68.2};
            std::array<double, 6> x03 = {423.6, -265.5, 651.2, 76.1, -180.0, 76.1};
            std::array<double, 6> x04 = {423.6, 34.5, 351.2, 81.3, -180.0, 81.3};

            std::vector<std::array<double, 6>> x0204c = {x02, x04};

            auto start = std::chrono::steady_clock::now();

            while (rclcpp::ok() && std::chrono::steady_clock::now() - start < std::chrono::seconds(60))
            {
                move_joint(JReady, 20, 20);
                move_joint(J1, 0, 0, 3);
                move_line(X3, velx, accx, 2.5);
                for (int i = 0; i < 1; i++)
                {
                    move_line(X2, velx, accx, 2.5, 50, DR_BASE, DR_MV_MOD_ABS);
                    move_line(X1, velx, accx, 1.5, 50, DR_BASE, DR_MV_MOD_ABS);
                    move_line(X0, velx, accx, 2.5);
                    move_line(X1, velx, accx, 2.5, 50, DR_BASE, DR_MV_MOD_ABS);
                    move_line(X2, velx, accx, 1.5, 50, DR_BASE, DR_MV_MOD_ABS);
                    move_line(X3, velx, accx, 2.5, 50, DR_BASE, DR_MV_MOD_ABS);
                }
                move_joint(J00, 60, 60, 6);
                move_joint(J01r, 0, 0, 2, 100, DR_MV_MOD_ABS);
                move_joint(J02r, 0, 0, 2, 50, DR_MV_MOD_ABS);
                move_joint(J03r, 0, 0, 2);

                move_joint(J04r, 0, 0, 1.5);
                move_joint(J04r1, 0, 0, 2, 50, DR_MV_MOD_ABS);
                move_joint(J04r2, 0, 0, 4, 50, DR_MV_MOD_ABS);
                move_joint(J04r3, 0, 0, 4, 50, DR_MV_MOD_ABS);
                move_joint(J04r4, 0, 0, 2);

                move_joint(J05r, 0, 0, 2.5, 100, DR_MV_MOD_ABS);
                move_line(dREL1, velx, accx, 1, 50, DR_TOOL, DR_MV_MOD_ABS);
                move_line(dREL2, velx, accx, 1.5, 100, DR_TOOL, DR_MV_MOD_ABS);

                move_joint(J07r, 60, 60, 1.5, 100, DR_MV_MOD_ABS);
                move_joint(J08r, 60, 60, 2);

                move_joint(JEnd, 60, 60, 4);
            }
        }
        state[2] = 0; // Reset state after execution
    }

    void reset()
    {
        // Implement the reset logic here
        RCLCPP_INFO(this->get_logger(), "Resetting robot to initial state.");
        // You can call move_joint or move_line with initial positions if needed
        move_joint({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    }

    void move_joint(const std::array<double, 6> &joint_pose,
                    double vel = 50.0,
                    double acc = 50.0,
                    double time = 0.0,
                    double radius = 0.0,
                    int mode = 0) 
    {
        auto request = std::make_shared<dsr_msgs2::srv::MoveJoint::Request>();
        request->pos = joint_pose;
        request->vel = vel;
        request->acc = acc;
        request->time = time;
        request->radius = radius;
        request->mode = mode; 
        auto result = client_movej_->async_send_request(request);

        if (result.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
        {
            RCLCPP_INFO(this->get_logger(), "MoveJoint thành công.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "MoveJoint thất bại.");
        }
    }

    void move_line(const std::array<double, 6> &cartesian_pose,
                   std::array<double, 2> vel = {100.0, 0},
                   std::array<double, 2> acc = {50.0, 0},
                   double time = 0.0,
                   double radius = 0.0,
                   int ref = 0,
                   int mode = 0) // DR_BASE
    {
        auto request = std::make_shared<dsr_msgs2::srv::MoveLine::Request>();
        request->pos = cartesian_pose;
        request->vel = vel;
        request->acc = acc;
        request->time = time;
        request->radius = radius;
        request->ref = ref;
        request->mode = mode;

        auto result = client_movel_->async_send_request(request);

        if (result.wait_for(std::chrono::seconds(5)) == std::future_status::ready)
            RCLCPP_INFO(this->get_logger(), "MoveLine thành công.");
        else
            RCLCPP_ERROR(this->get_logger(), "MoveLine thất bại.");
    }
    void move_jointx(const std::array<double, 6>& pos,
                 double vel = 50.0,
                 double acc = 50.0,
                 double time = 0.0,
                 double radius = 0.0,
                 int mod = 0,
                 int ref = 0)

    {
        // Tạo request
        auto request = std::make_shared<dsr_msgs2::srv::MoveJointx::Request>();

        // Gán dữ liệu
        request->pos = pos;
        request->vel = vel;
        request->acc = acc;
        request->time = time;
        request->radius = radius;
        request->ref = ref;
        request->mode = mod;

        // Gửi request
        auto result = client_movej_x->async_send_request(request);

        // Chờ kết quả
        if (result.wait_for(std::chrono::seconds(10)) == std::future_status::ready)
        {
            RCLCPP_INFO(this->get_logger(), "MoveJointx thành công.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "MoveJointx thất bại.");
        }
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // ✅ Bắt buộc dùng make_shared
    auto node = std::make_shared<ControllerNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}