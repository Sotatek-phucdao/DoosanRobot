# 🤖 Dự án Điều Khiển Robot Doosan với ROS2 (C++)

Dự án này cung cấp giao diện điều khiển robot Doosan sử dụng ROS2 bằng ngôn ngữ C++. Bao gồm các chức năng như điều khiển joint, mô phỏng trong RViz2 và thực thi lệnh theo chuỗi chuyển động.

---

## 🔧 Hướng dẫn cài đặt

Để cài đặt chương trình cần thực hiện các bước sau:


### 1. Cài đặt ROS2

Tùy theo phiên bản Ubuntu, bạn có thể chọn một bản phù hợp. Dưới đây là hướng dẫn cài đặt ROS2 **Humble Hawksbill** trên Ubuntu 22.04:

```
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo tee /etc/apt/keyrings/ros.asc > /dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros.asc] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```



### 2. Cài đặt Doosan Robot

Làm theo hướng dẫn tại repository chính thức của Doosan:

👉 https://github.com/DoosanRobotics/doosan-robot2

Thêm ROS2 vào cấu hình shell: 

```
source ~/ros2_ws/install/setup.bash
```

### 3. Cài đặt và build mã nguồn
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Sotatek-phucdao/DoosanRobot.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

Lưu ý: 📌 Lưu ý: Sau khi đã biên dịch toàn bộ workspace ít nhất một lần, bạn có thể chỉ biên dịch một gói cụ thể để tiết kiệm thời gian bằng lệnh:

```
colcon build --packages-select my_robot_project
```


### 4.Chạy chương trình 
```
ros2 run my_robot_project robot_controller
```
