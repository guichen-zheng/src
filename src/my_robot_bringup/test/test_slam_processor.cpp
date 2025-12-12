#include <gtest/gtest.h>
#include <memory>
#include "my_robot_bringup/slam_processor.hpp"
#include "rclcpp/rclcpp.hpp"

class TestSlamProcessor : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<my_robot_bringup::SlamProcessor>(false);
    }
    
    void TearDown() override
    {
        node_.reset();
        rclcpp::shutdown();
    }
    
    std::shared_ptr<my_robot_bringup::SlamProcessor> node_;
};

TEST_F(TestSlamProcessor, MapInitialization)
{
    // 测试地图初始化
    auto map = node_->getCurrentMap();
    
    EXPECT_EQ(map.header.frame_id, "map");
    EXPECT_GT(map.info.width, 0);
    EXPECT_GT(map.info.height, 0);
    EXPECT_GT(map.info.resolution, 0.0);
    EXPECT_FALSE(map.data.empty());
}

TEST_F(TestSlamProcessor, ParameterSetup)
{
    // 测试参数设置
    auto laser_topic = node_->get_parameter("laser_topic");
    EXPECT_EQ(laser_topic.as_string(), "/scan");
    
    auto map_resolution = node_->get_parameter("map_resolution");
    EXPECT_GT(map_resolution.as_double(), 0.0);
    EXPECT_LT(map_resolution.as_double(), 1.0); // 分辨率应该在合理范围内
}

TEST_F(TestSlamProcessor, PoseHandling)
{
    // 测试位姿处理
    auto initial_pose = node_->getCurrentPose();
    
    // 初始位姿应该在原点附近
    EXPECT_NEAR(initial_pose.position.x, 0.0, 0.1);
    EXPECT_NEAR(initial_pose.position.y, 0.0, 0.1);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}