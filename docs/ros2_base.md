- [ros 一键安装](#ros-一键安装)
- [创建包](#创建包)
- [bag包](#bag包)
  - [节点创建](#节点创建)
  - [timer创建](#timer创建)
  - [service创建](#service创建)
  - [单线程/多线程 调度](#单线程多线程-调度)
- [ros参数设置](#ros参数设置)
- [ros cmake](#ros-cmake)
  - [cmake编译 / colcon编译](#cmake编译--colcon编译)

# ros 一键安装
```
wget http://fishros.com/install -O fishros && bash fishros
```
# 创建包

```
cd ~/ros2_humble_ws/src
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp std_msgs

ros2 pkg create my_robot --build-type ament_cmake --dependencies rclcpp std_msgs --node-name my_node

```

# bag包
> 录制bag
```shell
ros2 bag record -a //录制所有话题
ros2 bag record -o my_bag /chatter /odom  //录制指定话题，生成my_bag文件夹
ros2 bag record -a -d 10  //录制10秒后自动停止
ros2 bag record -a --max-bag-size 1024  每个bag文件最大1GB

ros2 bag record -a -s mcap  //sqlite 或者 mcap
ros2 bag record -a --compression-mode file --compression-format zstd  //文件压缩

ros2 bag record -a --max-bag-size 200000000 //每个bag文件最大200MB

```
> 播放bag
```shell
ros2 bag play my_bag //默认播放一次
ros2 bag play my_bag --loop  //循环播放
ros2 bag play my_bag --rate 2.0  //2倍速播放
ros2 bag play my_bag --topics /chatter  //只播放指定话题
ros2 bag play my_bag -s 5  //从第5秒开始播放
ros2 bag play my_bag --start-paused  //暂停播放
ros2 bag play my_bag --remap /old_topic:=/new_topic  //重映射话题
```

> 查看bag信息
```shell
ros2 bag info my_bag
```



**vscode配置**
```
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "/opt/ros/${ROS_DISTRO}/include/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c17",
            "cppStandard": "gnu++14",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```

## 节点创建

| 类型                  | 类/接口                                         | 是否生命周期管理 | 是否可组合 | 是否可多线程 | 典型应用      |
| ------------------- | -------------------| -------- | ----- | ------ | --------- |
| 普通 Node             | `rclcpp::Node`                               | 否        | 可     | 可      | 普通计算、简单节点 |
| Lifecycle Node      | `rclcpp_lifecycle::LifecycleNode`            | 是        | 可     | 可      | 驱动、控制器    |
| Composable Node     | `Node`/`LifecycleNode` + `rclcpp_components` | 取决于基类    | 是     | 可      | 高性能流水线    |
| Multi-threaded Node | `Node` + `MultiThreadedExecutor`             | 否        | 可     | 是      | 并发处理      |
| Managed Node        | LifecycleNode + 管理器                          | 是        | 可     | 可      | 系统调度管理    |


> 常用内置QOS

| QoS Profile          | 典型用途    | Reliability  | Depth | Durability       |
| -------------------- | ------- | ------------ | ----- | ---------------- |
| `SensorDataQoS`      | 高频传感器数据 | BEST\_EFFORT | 5     | VOLATILE         |
| `SystemDefaultsQoS`  | 系统默认    | RELIABLE     | 10    | VOLATILE         |
| `ServicesQoS`        | 服务      | RELIABLE     | 10    | VOLATILE         |
| `ParametersQoS`      | 参数更新    | RELIABLE     | 10    | TRANSIENT\_LOCAL |
| `ParameterEventsQoS` | 参数事件    | RELIABLE     | 10    | TRANSIENT\_LOCAL |


> 普通节点
```c++
继承rclcpp::Node

this->create_publisher<T>(topic, rclcpp::SystemDefaultsQoS());

this->create_subscription<T>(topic, rclcpp::SystemDefaultsQoS(), [](T_shared msg){

})
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
```

> 声明周期节点

rclcpp_lifecycle::LifecycleNode
支持 on_configure、on_activate、on_deactivate、on_cleanup 等回调

> composable Node
rclcpp_components::NodeFactory

- 多个节点可以装载到同一个进程中运行
- 节省内存、减少进程间通信开销
- 可运行在 component_container 中
```
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager <package_name> <plugin_name>

```

## timer创建

```c++
using namespace std::chrono_literals;
rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(
            500ms,
            [this]() {
                auto msg = std::make_shared<std_msgs::msg::String>();
                msg->data = "Hello ROS2 pointer!";
                publisher_->publish(msg);  // 传递指针
            }
        );
```

## service创建

> 服务端

```
services/srv/AddTwoInts.srv

int64 a
int64 b
---
int64 sum

```
```c++
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

auto server = node->create_service<std_srvs::srv::SetBool>(
    "set_bool_service",
    [](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
       std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        res->success = req->data;
        res->message = req->data ? "ON" : "OFF";
    });

```
> 客户端
```c++
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

auto client = node->create_client<std_srvs::srv::SetBool>("set_bool_service");`
auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
request->data = true;

auto result_future = client->async_send_request(request);

// 等待服务端可用
while (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(), "Waiting for service...");
}
// 等待响应
if (rclcpp::spin_until_future_complete(node, result_future) ==
    rclcpp::FutureReturnCode::SUCCESS)
{
    auto response = result_future.get();
    RCLCPP_INFO(node->get_logger(), "Success: %s, Message: %s",
                response->success ? "true" : "false",
                response->message.c_str());
} else {
    RCLCPP_ERROR(node->get_logger(), "Failed to call service set_bool_service");
}


```



## 单线程/多线程 调度

> 单线程

```c++
rclcpp::init(argc, argv);
auto node = std::make_shared<MyNode>();
rclcpp::spin(node);
rclcpp::shutdown();
return 0;
```
> 多线程

```c++
rclcpp::executors::MultiThreadedExecutor exec;
exec.add_node(node);
exec.spin();

```

> 调度组


# ros参数设置

| 服务名                                    | 服务类型                                         | 功能                    |
| -------------------------------------- | -------------------------------------------- | --------------------- |
| `/node_name/get_parameters`            | `rcl_interfaces/srv/GetParameters`           | 获取一个或多个参数值            |
| `/node_name/set_parameters`            | `rcl_interfaces/srv/SetParameters`           | 设置一个或多个参数值            |
| `/node_name/set_parameters_atomically` | `rcl_interfaces/srv/SetParametersAtomically` | 原子地设置多个参数，保证全部成功或全部失败 |
| `/node_name/list_parameters`           | `rcl_interfaces/srv/ListParameters`          | 列出节点所有参数（可按前缀过滤）      |


> **1. 声明参数**

```c++
declare_parameter<std::string>("robot_name", "default_bot");
```

> **2. 获取参数**
```c++
std::string robot_name = this->get_parameter("robot_name").as_string();
std::string robot_name = this->get_parameter_or("robot_name", robot_name, std::string("default_bot"));

```
> **3. 动态参数**
```c++
rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle;

cb_handle = this->add_on_set_parameters_callback(
  [this](const std::vector<rclcpp::Parameter> & params) {
    for (const auto & param : params) {
      if (param.get_name() == "gain") {
        gain_ = param.as_double();
      }
    }
    return rcl_interfaces::msg::SetParametersResult().set_successful(true);
  });

```
> **4. 示例**
```c++
#include "rclcpp/rclcpp.hpp"

class ParamNode : public rclcpp::Node {
public:
  ParamNode() : Node("param_node") {
    // 声明参数
    this->declare_parameter<std::string>("robot_name", "default_bot");
    this->declare_parameter<double>("gain", 1.0);

    // 读取参数
    robot_name_ = this->get_parameter("robot_name").as_string();
    gain_ = this->get_parameter("gain").as_double();

    RCLCPP_INFO(this->get_logger(), "robot_name: %s", robot_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "gain: %.2f", gain_);

    // 设置动态参数回调
    cb_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        for (const auto & param : params) {
          if (param.get_name() == "gain") {
            gain_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Updated gain: %.2f", gain_);
          }
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });
  }

private:
  std::string robot_name_;
  double gain_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr cb_handle_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParamNode>());
  rclcpp::shutdown();
  return 0;
}

```

| 类型            | C++ API             |
| ------------- | ------------------- |
| bool          | `as_bool()`         |
| int           | `as_int()`          |
| double        | `as_double()`       |
| string        | `as_string()`       |
| bool\_array   | `as_bool_array()`   |
| int\_array    | `as_int_array()`    |
| double\_array | `as_double_array()` |
| string\_array | `as_string_array()` |
| byte\_array   | `as_byte_array()`   |


> yaml 示例

```yaml
# 文件头必须指定 ros__parameters
my_node_name:
  ros__parameters:
    param_name1: value1
    param_name2: value2
    param_name3: [1.0, 2.0, 3.0]
    param_name4:
      sub_param1: true
      sub_param2: "hello"
node1:
  ros__parameters:
    param_a: 10
    param_b: "abc"

node2:
  ros__parameters:
    use_sim_time: true
    gains:
      kp: 1.0
      ki: 0.1
      kd: 0.01

```

| YAML 表达方式           | ROS 2 参数类型                  | 示例                               |
| ------------------- | --------------------------- | -------------------------------- |
| `true` / `false`    | bool                        | `enabled: true`                  |
| `42`                | int64                       | `count: 42`                      |
| `3.14`              | double                      | `gain: 3.14`                     |
| `"text"` / `'text'` | string                      | `name: "robot1"`                 |
| `[1, 2, 3]`         | 数组 (int/double/string/bool) | `values: [0.1, 0.2, 0.3]`        |
| 嵌套字典                | map                         | `pid: {p: 1.0, i: 0.1, d: 0.01}` |

```yaml
my_controller:
  ros__parameters:
    joints: ["joint1", "joint2", "joint3"]
    gains:
      joint1: {p: 100.0, i: 0.1, d: 1.0}
      joint2: {p: 120.0, i: 0.2, d: 1.5}
      joint3: {p: 80.0, i: 0.05, d: 0.5}

```

```yaml
joints:
  - joint1
  - joint2
  - joint3

numbers:
  - 10
  - 20
  - 30

nested_objects:
  - name: motor1
    gain: 1.0
  - name: motor2
    gain: 2.0
controller:
  ros__parameters:
    enabled: true
    joint_names: ["joint1", "joint2", "joint3"]

    # 也可以分行写
    velocities:
      - 0.1
      - 0.2
      - 0.3

    # 嵌套数组
    waypoints:
      - [0.0, 0.0]
      - [1.0, 0.5]
      - [2.0, 1.0]

```

> CLI操作
>
```
# 列出参数
ros2 param list /my_node

# 获取参数
ros2 param get /my_node my_int

# 设置参数
ros2 param set /my_node my_int 42

# 导出参数
ros2 param dump /my_node > my_params.yaml

```

> **5. 跨节点调用**

> set
```c++
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/parameter.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("param_client_node");

    // 创建 SetParameters 服务客户端
    auto client = node->create_client<rcl_interfaces::srv::SetParameters>("/param_node/set_parameters");

    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Service not available!");
        return 1;
    }

    // 构造参数
    rcl_interfaces::msg::Parameter param;
    param.name = "int_param";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    param.value.integer_value = 42;

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(param);

    // 异步发送请求
    auto result = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        for (auto &r : result.get()->results)
        {
            RCLCPP_INFO(node->get_logger(), "Set parameter success=%d", r.successful);
        }
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to call set_parameters service");
    }

    rclcpp::shutdown();
    return 0;
}

```

> get

```c++
#include "rclcpp/rclcpp.hpp"

class ParamNode : public rclcpp::Node
{
public:
    ParamNode() : Node("param_node")
    {
        this->declare_parameter<int>("int_param", 10);
        this->declare_parameter<std::string>("string_param", "hello");
        RCLCPP_INFO(this->get_logger(), "param_node started with int_param=10, string_param='hello'");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamNode>());
    rclcpp::shutdown();
    return 0;
}
```
> 流程

- 节点 A 声明参数并启动。
- 节点 B 通过 /param_node/get_parameters 获取参数值。
- 节点 B 通过 /param_node/set_parameters 修改参数。
- 节点 A 的回调监听到参数变化并打印日志。


# ros cmake

>常用cmake

```cmake
cmake_minimum_required(VERSION 3.16)
project(hand_control_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)  # 继续允许 FindPythonInterp / FindPythonLibs
endif()

#生成调试信息
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -rdynamic -fno-omit-frame-pointer -funwind-tables")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic -fno-omit-frame-pointer -funwind-tables")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Threads REQUIRED)


include_directories(include)
file(GLOB SRC src/*.cpp)


add_executable(${PROJECT_NAME}_node ${SRC})
# 连接依赖库
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_include_directories(${PROJECT_NAME}_node PRIVATE )
target_link_libraries(${PROJECT_NAME}_node boost_stacktrace_backtrace backtrace dl glog gflags)
target_link_libraries(${PROJECT_NAME} PRIVATE Threads::Threads)

file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
file(GLOB SRV_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

```

> **ament_target_dependencies()**

这是 ament_cmake 提供的 ROS2 宏。
作用是给 target 添加 ROS 依赖的库和头文件，并且自动处理 include 路径、编译选项、库依赖。

示例：
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(talker src/talker.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

```
等价于：
```cmake
target_include_directories(talker PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
target_link_libraries(talker ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

```

👉 可以看出，ament_target_dependencies() 就是帮你省去了一大堆 include_directories 和 target_link_libraries 的繁琐写法。

```cmake
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_node src/camera_node.cpp)

# ROS 依赖
ament_target_dependencies(camera_node rclcpp sensor_msgs)

# 第三方库
target_link_libraries(camera_node ${OpenCV_LIBRARIES})

```

## cmake编译 / colcon编译

> colcon编译
>

```bash
colcon build
colcon build --packages-select my_robot_pkg --merge-install
colcon build --symlink-install  //符号连接
colcon build --symlink-install --merge-install
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

```

> cmake编译
```bash
mkdir -p build
cd build
cmake ../src/my_robot_pkg -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --target all
cmake --install .

```