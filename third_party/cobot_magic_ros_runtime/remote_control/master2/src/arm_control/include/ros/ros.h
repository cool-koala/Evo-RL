#pragma once

#include <chrono>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros {

std::shared_ptr<rclcpp::Node> get_node();
std::vector<std::shared_ptr<void>>& keep_alive();

class Subscriber {
public:
    Subscriber() = default;
    ~Subscriber();
    Subscriber(const Subscriber&) = default;
    Subscriber& operator=(const Subscriber&) = default;

private:
    void* storage_[2] = {nullptr, nullptr};
};

class Publisher {
public:
    Publisher() = default;

    template <typename Msg>
    void publish(const Msg& msg) const {
        if (publish_fn_) {
            publish_fn_(&msg);
        }
    }

private:
    template <typename Msg>
    friend class PublisherFactory;
    friend class NodeHandle;

    std::function<void(const void*)> publish_fn_;
};

struct SubscribeOptions {};
struct TransportHints {};

class NodeHandle {
public:
    NodeHandle() = default;

    template <typename T>
    void param(const std::string& name, T& value, const T& default_value) {
        auto node = get_node();
        if (!node->has_parameter(name)) {
            node->template declare_parameter<T>(name, default_value);
        }
        value = node->get_parameter(name).template get_value<T>();
    }

    template <typename Msg>
    Publisher advertise(const std::string& topic, uint32_t queue_size) {
        auto publisher = get_node()->template create_publisher<Msg>(topic, queue_size);
        Publisher wrapper;
        wrapper.publish_fn_ = [publisher](const void* msg) {
            publisher->publish(*static_cast<const Msg*>(msg));
        };
        keep_alive().push_back(publisher);
        return wrapper;
    }

    template <typename Msg, typename Callback>
    Subscriber subscribe(const std::string& topic, uint32_t queue_size, Callback&& callback) {
        auto subscription = get_node()->template create_subscription<Msg>(
            topic,
            queue_size,
            std::forward<Callback>(callback));
        keep_alive().push_back(subscription);
        return Subscriber();
    }

    Subscriber subscribe(SubscribeOptions& options);
};

class Rate {
public:
    explicit Rate(double hz) : rate_(hz) {}
    void sleep() { rate_.sleep(); }

private:
    rclcpp::Rate rate_;
};

struct Time {
    static builtin_interfaces::msg::Time now();
};

void init(int argc, char** argv, const std::string& node_name);
bool ok();
void spinOnce();
namespace serialization {
void throwStreamOverrun();
}

namespace console {
struct FilterBase {};
struct LogLocation {};
extern bool g_initialized;

namespace levels {
enum Level { Debug = 0, Info, Warn, Error, Fatal, Count };
}

void initialize();
void initializeLogLocation(LogLocation* loc, const std::string& name, levels::Level level);
void setLogLocationLevel(LogLocation* loc, levels::Level level);
bool checkLogLocationEnabled(LogLocation* loc);
void print(FilterBase* filter, void* logger, levels::Level level, const char* file, int line,
           const char* function, const char* fmt, ...);
}

}  // namespace ros

#define ROS_INFO(...)  \
    do {               \
        std::printf(__VA_ARGS__); \
        std::printf("\n");       \
    } while (0)

#define ROS_WARN(...)  \
    do {               \
        std::fprintf(stderr, __VA_ARGS__); \
        std::fprintf(stderr, "\n");       \
    } while (0)

#define ROS_ERROR(...) \
    do {               \
        std::fprintf(stderr, __VA_ARGS__); \
        std::fprintf(stderr, "\n");       \
    } while (0)
