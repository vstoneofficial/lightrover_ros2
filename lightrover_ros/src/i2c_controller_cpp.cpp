#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "lightrover_interface/srv/wrc201_msg.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr uint8_t DEV_ADDR = 0x10;
constexpr size_t MAP_SIZE = 0x100;
constexpr uint8_t MU16_SD_VI = 0x12;
constexpr uint8_t MU8_O_EN = 0x10;
constexpr uint8_t MU8_TRIG = 0x11;
constexpr uint8_t MS32_A_POS0 = 0x48;
constexpr uint8_t MS32_A_POS1 = 0x4C;
constexpr uint8_t MS32_M_POS0 = 0x60;
constexpr uint8_t MS32_M_POS1 = 0x64;
constexpr uint8_t MOTOR_TRIGGER_VALUE = 0x03;

int i2c_smbus_access(int file, char read_write, uint8_t command, int size, union i2c_smbus_data * data)
{
  i2c_smbus_ioctl_data args;
  args.read_write = read_write;
  args.command = command;
  args.size = size;
  args.data = data;
  return ioctl(file, I2C_SMBUS, &args);
}

int64_t pack_two_signed32(int32_t low_value, int32_t high_value)
{
  uint8_t bytes[8];
  std::memcpy(bytes, &low_value, 4);
  std::memcpy(bytes + 4, &high_value, 4);
  int64_t packed = 0;
  std::memcpy(&packed, bytes, 8);
  return packed;
}

std::array<int32_t, 2> unpack_two_signed32(int64_t value)
{
  std::array<int32_t, 2> out{};
  std::memcpy(out.data(), &value, 8);
  return out;
}
}  // namespace

class VsWrc201I2c
{
public:
  explicit VsWrc201I2c(uint8_t dev_addr)
  : dev_addr_(dev_addr)
  {
    fd_ = open("/dev/i2c-1", O_RDWR);
    if (fd_ < 0) {
      throw std::runtime_error("failed to open /dev/i2c-1");
    }
    if (ioctl(fd_, I2C_SLAVE, dev_addr_) < 0) {
      close(fd_);
      fd_ = -1;
      throw std::runtime_error("failed to set I2C_SLAVE address");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ~VsWrc201I2c()
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  void init_memmap(double cut_off_level)
  {
    const auto initial = initial_memmap();
    write_memmap(MU8_O_EN, initial, 0xF0);
    read_all();
    const int cut_off_hex = static_cast<int>((cut_off_level / 3.3) * 0xfff);
    write_s16map(MU16_SD_VI, cut_off_hex);
  }

  void write_1_byte(uint8_t addr, int64_t data)
  {
    union i2c_smbus_data smbus_data;
    smbus_data.byte = static_cast<uint8_t>(data);
    if (i2c_smbus_access(fd_, I2C_SMBUS_WRITE, addr, I2C_SMBUS_BYTE_DATA, &smbus_data) < 0) {
      throw std::runtime_error("I2C write byte failed");
    }
  }

  void write_2_byte(uint8_t addr, int64_t data)
  {
    const uint64_t value = static_cast<uint64_t>(data);
    for (int i = 0; i < 2; ++i) {
      write_1_byte(addr + i, (value >> (i * 8)) & 0xff);
    }
  }

  void write_4_byte(uint8_t addr, int64_t data)
  {
    const uint64_t value = static_cast<uint64_t>(data);
    for (int i = 0; i < 4; ++i) {
      write_1_byte(addr + i, (value >> (i * 8)) & 0xff);
    }
  }

  void write_memmap(uint8_t addr, const std::vector<uint8_t> & data_array, size_t length)
  {
    if (length == 0) {
      throw std::runtime_error("I2C write_memmap length must be positive");
    }
    for (size_t i = 0; i < length; ++i) {
      write_1_byte(addr + i, data_array.at(i));
    }
  }

  void read_memmap(uint8_t addr, size_t length)
  {
    const uint8_t read_addr = addr - 1;
    for (size_t i = 0; i < length + 1; ++i) {
      const uint8_t target_addr = read_addr + i;
      const uint8_t value = read_byte_data(target_addr);
      if (i != 0) {
        memmap_[target_addr] = value;
      }
    }
  }

  void read_multi_memmap(uint8_t addr, size_t length)
  {
    auto read_data = read_i2c_block_data(addr, length);
    bool all_ff = true;
    for (const auto byte : read_data) {
      if (byte != 0xff) {
        all_ff = false;
        break;
      }
    }
    if (all_ff) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      read_data = read_i2c_block_data(addr, length);
    }
    for (size_t i = 0; i < read_data.size(); ++i) {
      memmap_[addr + i] = read_data[i];
    }
  }

  void read_all()
  {
    read_memmap(0x00, 64);
    read_memmap(0x40, 64);
    read_memmap(0x80, 20);
  }

  void send_write_map()
  {
    send_write_range(0x12, 0x90);
    send_write_range(0x0e, 0x12);
  }

  uint8_t read_s8map(uint8_t addr) const
  {
    return memmap_[addr];
  }

  uint16_t read_s16map(uint8_t addr) const
  {
    return (static_cast<uint16_t>(memmap_[addr + 1]) << 8) | memmap_[addr];
  }

  uint32_t read_s32map(uint8_t addr) const
  {
    return (static_cast<uint32_t>(memmap_[addr + 3]) << 24) |
           (static_cast<uint32_t>(memmap_[addr + 2]) << 16) |
           (static_cast<uint32_t>(memmap_[addr + 1]) << 8) |
           memmap_[addr];
  }

  uint16_t write_s16map(uint8_t addr, int data)
  {
    memmap_[addr] = static_cast<uint8_t>(data & 0xff);
    memmap_[addr + 1] = static_cast<uint8_t>((data & 0xff00) >> 8);
    write_flag_[addr] = 1;
    write_flag_[addr + 1] = 1;
    return read_s16map(addr);
  }

private:
  uint8_t read_byte_data(uint8_t addr)
  {
    union i2c_smbus_data data;
    if (i2c_smbus_access(fd_, I2C_SMBUS_READ, addr, I2C_SMBUS_BYTE_DATA, &data) < 0) {
      throw std::runtime_error("I2C read byte failed");
    }
    return data.byte & 0xff;
  }

  std::vector<uint8_t> read_i2c_block_data(uint8_t addr, size_t length)
  {
    if (length > I2C_SMBUS_BLOCK_MAX) {
      throw std::runtime_error("I2C block read length exceeds SMBus maximum");
    }
    union i2c_smbus_data data;
    data.block[0] = static_cast<uint8_t>(length);
    if (i2c_smbus_access(fd_, I2C_SMBUS_READ, addr, I2C_SMBUS_I2C_BLOCK_DATA, &data) < 0) {
      throw std::runtime_error("I2C block read failed");
    }
    std::vector<uint8_t> out(length);
    for (size_t i = 0; i < length; ++i) {
      out[i] = data.block[i + 1] & 0xff;
    }
    return out;
  }

  static std::vector<uint8_t> initial_memmap()
  {
    std::vector<uint8_t> initial(0xF0, 0x00);
    const std::array<uint8_t, 32> prefix{{
      0x00, 0x00, 0xed, 0x05, 0xff, 0x01, 0x64, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0xff,
      0x00, 0xff, 0x00, 0xff, 0x00, 0x10, 0x00, 0x10
    }};
    std::copy(prefix.begin(), prefix.end(), initial.begin());
    return initial;
  }

  void send_write_range(size_t begin, size_t end)
  {
    for (size_t i = begin; i < end; ++i) {
      if (!write_flag_[i]) {
        continue;
      }

      const size_t head_addr = i;
      size_t length = 0;
      while (i < MAP_SIZE && write_flag_[i]) {
        write_flag_[i] = 0;
        ++length;
        ++i;
      }

      std::vector<uint8_t> write_map(length);
      for (size_t offset = 0; offset < length; ++offset) {
        write_map[offset] = memmap_[head_addr + offset];
      }
      write_memmap(static_cast<uint8_t>(head_addr), write_map, length);

      if (i > 0) {
        --i;
      }
    }
  }

  int fd_{-1};
  uint8_t dev_addr_{DEV_ADDR};
  std::array<uint8_t, MAP_SIZE> memmap_{};
  std::array<uint8_t, MAP_SIZE> write_flag_{};
};

class Wrc201I2cServer : public rclcpp::Node
{
public:
  Wrc201I2cServer()
  : Node("wrc201_i2c_server"), i2c_(DEV_ADDR)
  {
    if (!initialize_i2c()) {
      RCLCPP_ERROR(this->get_logger(), "I2C initialization failed, node stays alive for retry.");
    } else {
      RCLCPP_INFO(this->get_logger(), "I2C initialization success.");
    }

    srv_ = this->create_service<lightrover_interface::srv::Wrc201Msg>(
      "wrc201_i2c",
      std::bind(&Wrc201I2cServer::handle_wrc201_i2c, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Service is start");
  }

private:
  bool initialize_i2c(int retries = 5, std::chrono::milliseconds delay = std::chrono::milliseconds(200))
  {
    for (int i = 0; i < retries; ++i) {
      try {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        i2c_.read_all();
        i2c_.init_memmap(2.0);
        i2c_.send_write_map();
        return true;
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "I2C init failed (%d/%d): %s", i + 1, retries, e.what());
        std::this_thread::sleep_for(delay);
      }
    }
    return false;
  }

  static int32_t to_signed_32(uint32_t value)
  {
    return static_cast<int32_t>(value);
  }

  void handle_wrc201_i2c(
    const std::shared_ptr<lightrover_interface::srv::Wrc201Msg::Request> req,
    std::shared_ptr<lightrover_interface::srv::Wrc201Msg::Response> response)
  {
    try {
      const uint8_t addr = static_cast<uint8_t>(req->addr);
      const uint8_t length = static_cast<uint8_t>(req->length);

      if (req->cmd == "motor_w") {
        const auto motor_cmd = unpack_two_signed32(req->data);
        i2c_.write_4_byte(MS32_A_POS0, motor_cmd[0]);
        i2c_.write_4_byte(MS32_A_POS1, motor_cmd[1]);
        i2c_.write_1_byte(MU8_TRIG, MOTOR_TRIGGER_VALUE);
        response->read_data = 1;
        return;
      }

      if (req->cmd == "enc_pair_r") {
        i2c_.read_multi_memmap(MS32_M_POS0, 8);
        const int32_t enc_a = to_signed_32(i2c_.read_s32map(MS32_M_POS0));
        const int32_t enc_b = to_signed_32(i2c_.read_s32map(MS32_M_POS1));
        response->read_data = pack_two_signed32(enc_a, enc_b);

        if (read_debug_counter_ % 20 == 0) {
          RCLCPP_INFO(this->get_logger(), "I2C paired encoder read A=%d B=%d", enc_a, enc_b);
        }
        ++read_debug_counter_;
        return;
      }

      if (req->cmd == "w") {
        if (length == 4) {
          i2c_.write_4_byte(addr, req->data);
        } else if (length == 2) {
          i2c_.write_2_byte(addr, req->data);
        } else if (length == 1) {
          i2c_.write_1_byte(addr, req->data);
        } else {
          RCLCPP_WARN(this->get_logger(), "Unsupported write length: %u", length);
          response->read_data = -1;
          return;
        }
        response->read_data = 1;
        return;
      }

      if (req->cmd == "s") {
        i2c_.send_write_map();
        response->read_data = 1;
        return;
      }

      if (req->cmd == "rm") {
        i2c_.read_all();
        response->read_data = 1;
        return;
      }

      if (req->cmd == "r") {
        i2c_.read_multi_memmap(addr, length);
        if (length == 4) {
          response->read_data = to_signed_32(i2c_.read_s32map(addr));
        } else if (length == 2) {
          response->read_data = i2c_.read_s16map(addr);
        } else if (length == 1) {
          response->read_data = i2c_.read_s8map(addr);
        } else {
          RCLCPP_WARN(this->get_logger(), "Unsupported read length: %u", length);
          response->read_data = 0;
          return;
        }

        if ((addr == MS32_M_POS0 || addr == MS32_M_POS1) && read_debug_counter_ % 20 == 0) {
          RCLCPP_INFO(
            this->get_logger(), "I2C read addr=0x%02X length=%u data=%ld",
            addr, length, response->read_data);
        }
        if (addr == MS32_M_POS0 || addr == MS32_M_POS1) {
          ++read_debug_counter_;
        }
        return;
      }

      RCLCPP_WARN(this->get_logger(), "Unknown I2C command: %s", req->cmd.c_str());
      response->read_data = -1;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "I2C service error: %s", e.what());
      response->read_data = -1;
    }
  }

  VsWrc201I2c i2c_;
  rclcpp::Service<lightrover_interface::srv::Wrc201Msg>::SharedPtr srv_;
  int read_debug_counter_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Wrc201I2cServer>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("wrc201_i2c_server"), "Failed to start I2C server: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
