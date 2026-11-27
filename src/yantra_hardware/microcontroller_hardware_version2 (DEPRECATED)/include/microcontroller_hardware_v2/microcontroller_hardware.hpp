#ifndef YANTRA_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_
#define YANTRA_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <string>
#include <iostream>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/char.hpp"

#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <string>
#include <sstream>
#include <cstdlib>

#include <fstream>
#include <cstdlib>
using namespace std;


// Detect CPU architecture
#if defined(__x86_64__) || defined(_M_X64)
    #include <immintrin.h>  // AVX for x86_64
    #define USE_SIMD 1
#else
    #define USE_SIMD 0  // No AVX on ARM (Raspberry Pi, Jetson)
#endif


namespace yantra_hardware {

  // IMU Data Struct (Fixed Memory Layout)
  struct Data {
    float U, G, M;  // Orientation
    float D, I, Y;  // Gyro
    float P, C, T;  // Acceleration
    int B;          // Status bit
  };
  // Memory Pool for Efficient Allocation
#define MEMORY_POOL_SIZE 10
#define RING_BUFFER_SIZE 1024
  class MemoryPool {
  private:
    Data pool[MEMORY_POOL_SIZE];
    int index = 0;
    std::mutex mtx;

  public:
    Data* allocate() {
      std::lock_guard<std::mutex> lock(mtx);
      Data* ptr = &pool[index];
      index = (index + 1) % MEMORY_POOL_SIZE;
      return ptr;
    }
  };

  // Ring Buffer for Serial Data
  class RingBuffer {
  private:
    char buffer[RING_BUFFER_SIZE];
    int head = 0, tail = 0;
    std::mutex mtx;

  public:
    bool isEmpty() { return head == tail; }
    bool isFull() { return ((head + 1) % RING_BUFFER_SIZE) == tail; }

    void push(char c) {
      std::lock_guard<std::mutex> lock(mtx);
      if (!isFull()) {
        buffer[head] = c;
        head = (head + 1) % RING_BUFFER_SIZE;
      }
    }

    std::string extractMessage() {
      std::lock_guard<std::mutex> lock(mtx);
      std::string msg;
      bool inMessage = false;
      int tempTail = tail;

      while (tempTail != head) {
        char c = buffer[tempTail];
        tempTail = (tempTail + 1) % RING_BUFFER_SIZE;

        if (c == '$') {  // Start marker
          msg.clear();
          inMessage = true;
        }
        if (inMessage) {
          msg += c;
        }
        if (c == '\n') {  // End marker
          tail = tempTail;
          return msg;
        }
      }
      return "";
    }
  };



  class MicrocontrollerHardware : public hardware_interface::SensorInterface {
  public:
    RCLCPP_SHARED_PTR_DEFINITIONS(MicrocontrollerHardware);


    int initSerial(const char* port);
    int configureSerial(int fd);
    std::string readSerial(int fd);
    bool parseMessage(const std::string& message, Data& data);
    Data* parseData(const std::string& data, Data* imu);
    Data datas_;
    fd_set read_fds;



    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;


  private:
    rclcpp::Node::SharedPtr nh_;
    sensor_msgs::msg::Imu imu_msg_;

    std::shared_ptr<RingBuffer> ringBuffer;
    std::shared_ptr<MemoryPool> memoryPool;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr button_pub_;

    std::string portname_;
    int baudrate_;
    int waitTimeout;
    int timeOut_ctr = 0;
    bool broadcast = broadcast;
    bool reconnect = false;
    bool dev_connected = false;

    bool data_ready = false;
    float roll, pitch, yaw, gyroRoll, gyroPitch, gyroYaw, accelX, accelY, accelZ, temp_data;
    std::string read_buffer;
    char buffer;

    int fd;                   // File descriptor for the port
    std::string inputBuffer;  // Accumulate incoming data here
    char readBuf[128];        // Temporary read buffer

    float button;
    char type;
    float value;

    std::vector<double> hw_states_;
    std::array<double, 4> orientation_;
    std::array<double, 3> angular_velocity_;
    std::array<double, 3> linear_acceleration_;
    double button_msg_;

#define DEGREE2RADIAN 0.01745;
  };

}  // namespace yantra_hardware

#endif  // YANTRA_HARDWARE__MICROCONTROLLER_HARDWARE_HPP_
