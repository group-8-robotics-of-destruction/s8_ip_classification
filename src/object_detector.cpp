#include <ros/ros.h>
#include <s8_common_node/Node.h>
// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
// OTHER
#include <vector>


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1

#define NODE_NAME           		"s8_object_classification_node"
#define TOPIC_POINT_CLOUD   		"/s8/detectedObject"
#define TOPIC_EXTRACTED_OBJECTS		""

typedef pcl::PointXYZRGB PointT;
using namespace std;

class ObjectClassifier : public s8::Node 
{
	const int hz;

	ros::Subscriber point_cloud_subscriber;
	ros::Publisher point_cloud_publisher;
	pcl::PointCloud<PointT>::Ptr cloud;

	bool cloudInitialized;

public:
	ObjectClassifier(int hz) : hz(hz)
	{
		add_params();
		//printParams();
		point_cloud_subscriber = nh.subscribe(TOPIC_POINT_CLOUD, BUFFER_SIZE, &ObjectDetector::point_cloud_callback, this);
		point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2> (TOPIC_EXTRACTED_OBJECTS, BUFFER_SIZE);

		cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
		cloudInitialized = false;
	}

	void updateClass()
	{	
		if ( cloudInitialized == false)
			return;

		return;
	}

private:
	
	void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
	{
		pcl::fromROSMsg (*cloud_msg, *cloud);
		cloudInitialized = true;
	}

	void cloudPublish(pcl::PointCloud<PointT>::Ptr cloud_pub)
	{
		sensor_msgs::PointCloud2 output;
		pcl::toROSMsg(*cloud_pub, output);
		point_cloud_publisher.publish (output);
	}

	void add_params() 
    {
    }
};

int main(int argc, char **argv) {
    
    ros::init(argc, argv, NODE_NAME);

    ObjectClassifier classifier(HZ);
    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
    	ros::spinOnce();
        classifier.updateClass();
        loop_rate.sleep();
    }

    return 0;
}