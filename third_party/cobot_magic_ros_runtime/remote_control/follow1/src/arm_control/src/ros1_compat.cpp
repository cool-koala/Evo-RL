#include <ros/ros.h>
#include <ros/package.h>

#include <cstdarg>
#include <stdexcept>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ros {
namespace {
std::shared_ptr<rclcpp::Node> g_node;
std::vector<std::shared_ptr<void>> g_keep_alive;
}  // namespace

Subscriber::~Subscriber() = default;

std::shared_ptr<rclcpp::Node> get_node() {
    if (!g_node) {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        g_node = std::make_shared<rclcpp::Node>("arm_node");
    }
    return g_node;
}

std::vector<std::shared_ptr<void>>& keep_alive() {
    return g_keep_alive;
}

void init(int argc, char** argv, const std::string& node_name) {
    if (!rclcpp::ok()) {
        rclcpp::init(argc, argv);
    }
    g_node = std::make_shared<rclcpp::Node>(node_name);
}

bool ok() {
    return rclcpp::ok();
}

void spinOnce() {
    rclcpp::spin_some(get_node());
}

Subscriber NodeHandle::subscribe(SubscribeOptions& /*options*/) {
    return Subscriber();
}

builtin_interfaces::msg::Time Time::now() {
    return get_node()->now();
}

namespace serialization {
void throwStreamOverrun() {
    throw std::runtime_error("ROS1 serialization stream overrun in compatibility shim");
}
}  // namespace serialization

namespace console {
bool g_initialized = false;

void initialize() {
    g_initialized = true;
}

void initializeLogLocation(LogLocation* /*loc*/, const std::string& /*name*/, levels::Level /*level*/) {}
void setLogLocationLevel(LogLocation* /*loc*/, levels::Level /*level*/) {}
bool checkLogLocationEnabled(LogLocation* /*loc*/) {
    return true;
}

void print(FilterBase* /*filter*/, void* /*logger*/, levels::Level level, const char* file, int line,
           const char* function, const char* fmt, ...) {
    FILE* stream = level >= levels::Warn ? stderr : stdout;
    std::fprintf(stream, "[ros1_compat] %s:%d %s: ", file ? file : "unknown", line,
                 function ? function : "unknown");
    va_list args;
    va_start(args, fmt);
    std::vfprintf(stream, fmt, args);
    va_end(args);
    std::fprintf(stream, "\n");
}
}  // namespace console

namespace package {
std::string getPath(const std::string& package_name) {
    try {
        return ament_index_cpp::get_package_share_directory(package_name);
    } catch (const std::exception&) {
        return ".";
    }
}
}  // namespace package

}  // namespace ros
