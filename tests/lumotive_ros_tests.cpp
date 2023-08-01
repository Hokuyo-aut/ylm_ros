// ROS includes
#include <ros/ros.h>

#include <gtest/gtest.h>
#include <string>

// Message types
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_field_conversion.h>


// Global variables
sensor_msgs::PointCloud2 pointcloud_msg_org, pointcloud_msg_unorg;


// ******************************************************************************************
// Callbacks
// ******************************************************************************************
void pt_callback_org(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pointcloud_msg_org = *msg;
}

void pt_callback_unorg(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pointcloud_msg_unorg = *msg;
}


// ******************************************************************************************
// Tests organized cloud
// ******************************************************************************************
TEST(LumotiveRosTests, MessageHeaderTestOrg)
{
	ASSERT_EQ(pointcloud_msg_org.header.frame_id, "m20_lidar");
	ASSERT_EQ(pointcloud_msg_org.height, 10);
	ASSERT_EQ(pointcloud_msg_org.width, 1278);
	ASSERT_EQ(pointcloud_msg_org.is_dense, false);
}

TEST(LumotiveRosTests, XYZTestOrg)
{

	float x, y, z;
	for (uint32_t pixelIdx = 0; pixelIdx < pointcloud_msg_org.height * pointcloud_msg_org.width * pointcloud_msg_org.point_step; pixelIdx += pointcloud_msg_org.point_step)
	{
		 x = sensor_msgs::readPointCloud2BufferValue<sensor_msgs::PointField::FLOAT32, float>(&pointcloud_msg_org.data[pixelIdx]);
		 y = sensor_msgs::readPointCloud2BufferValue<sensor_msgs::PointField::FLOAT32, float>(&pointcloud_msg_org.data[pixelIdx+4]);
		 z = sensor_msgs::readPointCloud2BufferValue<sensor_msgs::PointField::FLOAT32, float>(&pointcloud_msg_org.data[pixelIdx+8]);

		 ASSERT_EQ(x, 6.0);
		 ASSERT_EQ(y, 7.0);
		 ASSERT_EQ(z, 8.0);
	}
}

TEST(LumotiveRosTests, SignalTestOrg)
{

	uint16_t signal;
	for (uint32_t pixelIdx = 0; pixelIdx < pointcloud_msg_org.height * pointcloud_msg_org.width * pointcloud_msg_org.point_step; pixelIdx += pointcloud_msg_org.point_step)
	{
		 signal = sensor_msgs::readPointCloud2BufferValue<sensor_msgs::PointField::UINT16, uint16_t>(&pointcloud_msg_org.data[pixelIdx+12]);

		 ASSERT_EQ(signal, 678);
	}
}

// ******************************************************************************************
// Tests unorganized cloud
// ******************************************************************************************
TEST(LumotiveRosTests, MessageHeaderTestUnorg)
{
	ASSERT_EQ(pointcloud_msg_unorg.header.frame_id, "m20_lidar");
	ASSERT_EQ(pointcloud_msg_unorg.height, 1);
	ASSERT_EQ(pointcloud_msg_unorg.width, 1278*10);
	ASSERT_EQ(pointcloud_msg_unorg.is_dense, true);
}

TEST(LumotiveRosTests, XYZTestUnorg)
{

	float x, y, z;
	for (uint32_t pixelIdx = 0; pixelIdx < pointcloud_msg_unorg.width * pointcloud_msg_unorg.point_step; pixelIdx += pointcloud_msg_unorg.point_step)
	{
		 x = sensor_msgs::readPointCloud2BufferValue<sensor_msgs::PointField::FLOAT32, float>(&pointcloud_msg_unorg.data[pixelIdx]);
		 y = sensor_msgs::readPointCloud2BufferValue<sensor_msgs::PointField::FLOAT32, float>(&pointcloud_msg_unorg.data[pixelIdx+4]);
		 z = sensor_msgs::readPointCloud2BufferValue<sensor_msgs::PointField::FLOAT32, float>(&pointcloud_msg_unorg.data[pixelIdx+8]);

		 ASSERT_EQ(x, 6.0);
		 ASSERT_EQ(y, 7.0);
		 ASSERT_EQ(z, 8.0);
	}
}

TEST(LumotiveRosTests, SignalTestUnorg)
{

	uint16_t signal;
	for (uint32_t pixelIdx = 0; pixelIdx < pointcloud_msg_unorg.width * pointcloud_msg_unorg.point_step; pixelIdx += pointcloud_msg_unorg.point_step)
	{
		 signal = sensor_msgs::readPointCloud2BufferValue<sensor_msgs::PointField::UINT16, uint16_t>(&pointcloud_msg_unorg.data[pixelIdx+12]);

		 ASSERT_EQ(signal, 678);
	}
}

int main(int argc, char **argv)
{

	// Init google test
	testing::InitGoogleTest(&argc, argv);

	// Init testing node
	ros::init(argc, argv, "lumotive_ros_tester");
  	ros::NodeHandle nh;
  	ros::NodeHandle param_nh("~");

  	// Subscribe to topics
	ros::Subscriber sub_org = nh.subscribe("/lumotive_ros/pointcloud_org", 1, pt_callback_org);
	ros::Subscriber sub_unorg = nh.subscribe("/lumotive_ros/pointcloud_unorg", 1, pt_callback_unorg);

	// Wait a few seconds to make sure that the publishing nodes are well started
	sleep(3);

	// Spin once to fetch one message on each callback
	ros::spinOnce();

	// Run tests
	return RUN_ALL_TESTS();
}