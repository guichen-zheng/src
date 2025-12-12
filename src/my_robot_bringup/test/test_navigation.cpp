#include <gtest/gtest.h>
#include <memory>
#include "my_robot_bringup/navigation_core.hpp"
#include "rclcpp/rclcpp.hpp"

class TestNavigationCore : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<my_robot_bringup::NavigationCore>(false); // 非仿真模式
    }
    
    void TearDown() override
    {
        node_.reset();
        rclcpp::shutdown();
    }
    
    std::shared_ptr<my_robot_bringup::NavigationCore> node_;
};

TEST_F(TestNavigationCore, Initialization)
{
    // 测试节点是否成功初始化
    EXPECT_NE(node_, nullptr);
    
    // 测试参数是否正确设置
    auto parameters = node_->list_parameters({}, 10);
    EXPECT_FALSE(parameters.names.empty());
}

TEST_F(TestNavigationCore, VelocityCommands)
{
    // 测试速度命令发布
    auto start_time = node_->now();
    
    // 等待一小段时间让节点完全初始化
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // 检查发布器是否存在
    auto publishers = node_->get_publishers_info();
    bool has_cmd_vel_publisher = false;
    for (const auto& pub : publishers) {
        if (pub.topic_name().find("cmd_vel") != std::string::npos) {
            has_cmd_vel_publisher = true;
            break;
        }
    }
    EXPECT_TRUE(has_cmd_vel_publisher);
}

TEST_F(TestNavigationCore, ParameterValidation)
{
    // 测试参数范围
    auto max_linear_param = node_->get_parameter("max_linear_speed");
    EXPECT_GT(max_linear_param.as_double(), 0.0);
    
    auto max_angular_param = node_->get_parameter("max_angular_speed");
    EXPECT_GT(max_angular_param.as_double(), 0.0);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}