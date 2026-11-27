#include "microcontroller_hardware_v2/microcontroller_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <sys/select.h>
#include <mutex>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE B1000000


#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace yantra_hardware
{
  hardware_interface::CallbackReturn MicrocontrollerHardware::on_init(const hardware_interface::HardwareInfo& info)
  {
    if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    ringBuffer = std::make_shared<RingBuffer>();
    memoryPool = std::make_shared<MemoryPool>();

    // Takes parameter defined in urdf file. Use xacro variable to pass parameter from launch file
    portname_ = info_.hardware_parameters["portname"];
    baudrate_ = stoi(info_.hardware_parameters["baudrate"]);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
    MicrocontrollerHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> MicrocontrollerHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces_conf;

    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &orientation_[0]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[1].name, &orientation_[1]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[2].name, &orientation_[2]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[3].name, &orientation_[3]));

    // Add state interfaces for angular velocity
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[4].name, &angular_velocity_[0]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[5].name, &angular_velocity_[1]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[6].name, &angular_velocity_[2]));

    // Add state interfaces for linear acceleration
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[7].name, &linear_acceleration_[0]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[8].name, &linear_acceleration_[1]));
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[9].name, &linear_acceleration_[2]));

    // Button State Interface
    state_interfaces_conf.emplace_back(hardware_interface::StateInterface(
      info_.sensors[1].name, info_.sensors[1].state_interfaces[0].name, &button_msg_));

    std::cout << info_.sensors[1].state_interfaces[0].name << " ----------------" << endl;

    return state_interfaces_conf;
  }

  hardware_interface::CallbackReturn
    MicrocontrollerHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Activating ...please wait...");
    Data datas_;
    read_fds;

    // fd = open(portname_.c_str(), O_RDWR | O_NOCTTY);
    // if (fd == -1) {
    //   std::cerr << "Error: Cannot open " << portname_ << "\n";
    //   return hardware_interface::CallbackReturn::ERROR;
    // }
    // else {
    //   RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully opened the port: %s", portname_.c_str());
    // }

    // MicrocontrollerHardware::configureSerial(fd);
    MicrocontrollerHardware::initSerial(portname_.c_str());

    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn
    MicrocontrollerHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    close(fd);
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MicrocontrollerHardware::on_error(const rclcpp_lifecycle::State& /*previous_state*/)
  {
    fd = open(portname_.c_str(), O_RDWR | O_NOCTTY);
    if (fd == -1) {
      std::cerr << "Error: Cannot open " << portname_ << "\n";
      RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "No Device Found, Searching for the Microcontroller");
      return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  int MicrocontrollerHardware::initSerial(const char* port) {
    fd = open(portname_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
      std::cerr << "Error opening " << port << "\n";
      return -1;
    }

    // Configure the serial port.
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    // options.c_cflag = CS8 | CLOCAL | CREAD;
    // options.c_iflag = IGNPAR;
    // options.c_oflag = 0;
    // options.c_lflag = 0;   // Set raw mode
    // Configure 8N1 (8 data bits, no parity, 1 stop bit)
    options.c_cflag = CS8 | CLOCAL | CREAD; // Set 8-bit, enable RX/TX, ignore modem control
    options.c_cflag &= ~PARENB;  // No parity
    options.c_cflag &= ~CSTOPB;  // 1 stop bit
    options.c_cflag &= ~CRTSCTS; // No hardware flow control
    // Configure input flags
    options.c_iflag = IGNPAR;       // Ignore framing/parity errors
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    // Configure output flags
    options.c_oflag &= ~OPOST; // Disable post-processing (raw output)
    // Configure local modes
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw mode, no echo, no signals

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);

    return fd;
  }

  // Serial Port Configuration (Compatible with ARM & x64)
  int MicrocontrollerHardware::configureSerial(int fd) {
    struct termios options;
    tcgetattr(fd, &options);

    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);

    options.c_cflag = CS8 | CLOCAL | CREAD;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return 0;
  }

  bool MicrocontrollerHardware::parseMessage(const std::string& message, Data& data) {
    // Ensure message starts with '$'
    if (message.empty() || message[0] != '$') {
      return false;
    }

    size_t pos = 1; // Skip the '$'
    // Loop until we hit the newline (or end of message)
    while (pos < message.size() && message[pos] != '\n') {
      // Each field starts with a label (e.g., 'U', 'G', etc.)
      char label = message[pos++];
      if (pos >= message.size()) break;

      // Find the next label character or the newline.
      // We assume labels are one of these: U, G, M, D, I, Y, P, C, T, B.
      size_t startNum = pos;
      while (pos < message.size() &&
        message[pos] != 'U' && message[pos] != 'G' && message[pos] != 'M' &&
        message[pos] != 'D' && message[pos] != 'I' && message[pos] != 'Y' &&
        message[pos] != 'P' && message[pos] != 'C' && message[pos] != 'T' &&
        message[pos] != 'B' && message[pos] != '\n') {
        pos++;
      }

      // Extract the substring containing the number.
      std::string numStr = message.substr(startNum, pos - startNum);
      float value = 0.0f;
      try {
        value = std::stof(numStr);
      }
      catch (const std::exception& e) {
        std::cerr << "Conversion error for token: " << numStr << std::endl;
        return false;
      }

      // Assign value to the corresponding field.
      switch (label) {
      case 'U': data.U = value; break;
      case 'G': data.G = value; break;
      case 'M': data.M = value; break;
      case 'D': data.D = value; break;
      case 'I': data.I = value; break;
      case 'Y': data.Y = value; break;
      case 'P': data.P = value; break;
      case 'C': data.C = value; break;
      case 'T': data.T = value; break;
      case 'B': data.B = static_cast<int>(value); break;
      default:
        std::cerr << "Unknown label: " << label << std::endl;
        return false;
      }
    }
    return true;
  }

  // Portable IMU Parser (Compatible with ARM & x64)
  Data* MicrocontrollerHardware::parseData(const std::string& data, Data* imu) {

    const char* ptr = data.c_str();
    char keys[9] = { 'U', 'G', 'M', 'D', 'I', 'Y', 'P', 'C', 'T' };
    float values[9] = { 0 };
    int intValue = 0;

    int idx = 0;
    while (*ptr) {
      if (*ptr == '$') {  // Start marker
        ptr++;
        continue;
      }
      char key = *ptr++;
      if (*ptr == '\0') break;
      if (key == 'B') {
        intValue = strtol(ptr, NULL, 10);  // Parse integer for B
      }
      else if (idx < 9) {
        values[idx++] = strtof(ptr, NULL);  // Parse float
      }

      RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Key: %c", key);

      while (*ptr && *ptr != '$' && *ptr != '\n' && *ptr != ' ') ptr++;
    }

#if USE_SIMD
    // __m256 parsed_values = _mm256_loadu_ps(values);  // Load 8 floats into SIMD
    // _mm256_storeu_ps(&data_->U, parsed_values);        // Store back into data_ struct
    // data_->T = values[8];  // Last value (9th) since AVX handles 8 at a time
    imu->U = values[0]; imu->G = values[1]; imu->M = values[2];
    imu->D = values[3]; imu->I = values[4]; imu->Y = values[5];
    imu->P = values[6]; imu->C = values[7]; imu->T = values[8];
#else
    imu->U = values[0]; imu->G = values[1]; imu->M = values[2];
    imu->D = values[3]; imu->I = values[4]; imu->Y = values[5];
    imu->P = values[6]; imu->C = values[7]; imu->T = values[8];
#endif
    // Button Data
    imu->B = intValue;
    RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "Button Value: %d", imu->B);

    return imu;
  }

  std::string MicrocontrollerHardware::readSerial(int fd) {
    char buffer[256];
    int bytesRead = ::read(fd, buffer, sizeof(buffer) - 1);

    if (bytesRead > 0) {
      buffer[bytesRead] = '\0';
      return std::string(buffer);
    }

    return "";
  }

  hardware_interface::return_type MicrocontrollerHardware::read(const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/)
  {
    char tempBuffer[256];
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);
    // struct timeval timeout = { 1, 0 };
    struct timeval timeout;
    timeout.tv_sec = 0;   // 0 seconds
    timeout.tv_usec = 10000; // 10 ms timeout


    int result = select(fd + 1, &read_fds, NULL, NULL, &timeout);
    std::string data = readSerial(fd);

    if (!data.empty()) {  // Data available
      Data* data_ = parseData(data, &datas_);
      std::cout << "IMU Parsed: U=" << data_->U << ", G=" << data_->G
        << ", M=" << data_->M << ", D=" << data_->D << "\n";

      // Use the parsed data as needed.
      roll = -1 * data_->U * DEGREE2RADIAN;
      imu_msg_.orientation.x = roll;
      orientation_[0] = roll;

      // pitch = Data.G * DEGREE2RADIAN;
      imu_msg_.orientation.y = pitch;
      orientation_[1] = pitch;

      yaw = -1 * data_->M * DEGREE2RADIAN;
      imu_msg_.orientation.z = yaw;
      orientation_[2] = yaw;

      gyroRoll = data_->D * DEGREE2RADIAN;
      imu_msg_.angular_velocity.x = gyroRoll;
      angular_velocity_[0] = gyroRoll;

      gyroPitch = -1 * data_->I * DEGREE2RADIAN;
      imu_msg_.angular_velocity.y = gyroPitch;
      angular_velocity_[1] = gyroPitch;

      gyroYaw = data_->Y * DEGREE2RADIAN;
      imu_msg_.angular_velocity.z = gyroYaw;
      angular_velocity_[2] = gyroYaw;

      accelX = data_->P;
      imu_msg_.linear_acceleration.x = accelX;
      linear_acceleration_[0] = accelX;

      accelY = data_->C;
      imu_msg_.linear_acceleration.y = accelY;
      linear_acceleration_[1] = accelY;

      accelZ = data_->T;
      imu_msg_.linear_acceleration.z = accelZ;
      linear_acceleration_[2] = accelZ;

      // button_msg_ = Data.B;
    }
    // else {
    //   RCLCPP_INFO(rclcpp::get_logger("MicrocontrollerHardware"), "No data available");
    // }
    return hardware_interface::return_type::OK;
  }

}  // namespace yantra_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(yantra_hardware::MicrocontrollerHardware, hardware_interface::SensorInterface)
