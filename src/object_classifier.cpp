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
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// OTHER
#include <vector>


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1

#define NODE_NAME           		"s8_object_classification_node"
#define TOPIC_POINT_CLOUD   		"/s8/detectedObject"
#define TOPIC_EXTRACTED_OBJECTS		"/s8/modifiedObject"

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
		point_cloud_subscriber = nh.subscribe(TOPIC_POINT_CLOUD, BUFFER_SIZE, &ObjectClassifier::point_cloud_callback, this);
		point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2> (TOPIC_EXTRACTED_OBJECTS, BUFFER_SIZE);

		cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
		cloudInitialized = false;
	}

	void updateClass()
	{	
		if ( cloudInitialized == false)
			return;
		getColors(cloud);

		cloudPublish(cloud);
	}

private:
	
	void getColors(pcl::PointCloud<PointT>::Ptr cloud_color)
	{
		uint8_t R_low = 255;
		uint8_t R_high = 0;
		uint8_t G_low = 255;
		uint8_t G_high = 0;
		uint8_t B_low = 255;
		uint8_t B_high = 0;
        double R_avg = 0;
        double G_avg = 0;
        double B_avg = 0;
        double R_acc = 0;
        double G_acc = 0;
        double B_acc = 0;
		for(int iter = 0; iter != cloud_color->points.size(); ++iter)
		{
		    //Whatever you want to do with the points
            uint8_t R = cloud_color->points[iter].r;
            uint8_t G = cloud_color->points[iter].g;
            uint8_t B = cloud_color->points[iter].b;
            ROS_INFO("R:, %d G: %d B: %d iter, %d", R, G, B, iter);
            R_acc += R;
            G_acc += G;
            B_acc += B;
/*            if (R_low > R)
                R_low = R;
		    if (R_high < R)
		    	R_high = R;
		    if (G_low > G)
		    	G_low = G;
		    if (G_high < G)
		    	G_high = G;
		    if (B_low > B)
		    	B_low = B;
		    if (B_high < B)
                B_high = B;*/
		}
        //ROS_INFO("R_low: %d R_high: %d G_low: %d G_high: %d B_low: %d B_high: %d", R_low, R_high, G_low, R_high, B_low, B_high);
        int size = cloud_color->points.size();
        R_avg =R_acc / size;
        G_avg =G_acc / size;
        B_avg =B_acc / size;
        ROS_INFO("R_avg: %lf G_avg: %lf B_avg: %lf R_acc: %lf G_acc: %lf B_acc: %lf cloud_size: %d", R_avg, G_avg, B_avg, R_acc, G_acc, B_acc, size);
	}

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
