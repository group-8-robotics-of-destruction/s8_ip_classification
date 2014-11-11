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
        cloudInitialized = false;
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
        float H_acc = 0;
        float S_acc = 0;
        float V_acc = 0;
        float H_avg = 0;
        float S_avg = 0;
        float V_avg = 0;
        for(int iter = 0; iter != cloud_color->points.size(); ++iter)
        {
            //Whatever you want to do with the points
            uint8_t R = cloud_color->points[iter].r;
            uint8_t G = cloud_color->points[iter].g;
            uint8_t B = cloud_color->points[iter].b;
            //ROS_INFO("R:, %d G: %d B: %d iter, %d", R, G, B, iter);
            R_acc += R;
            G_acc += G;
            B_acc += B;
            /*float H, S, V;
            RGB2HSV((float)R, (float)G, (float)B, H, S, V);
            H_acc += H;
            S_acc += S;
            V_acc += V;*/
            //ROS_INFO("H: %f, S: %f, V: %f", H, S, V);
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
        R_avg = R_acc / size;
        G_avg = G_acc / size;
        B_avg = B_acc / size;
        RGB2HSV((float)R_avg, (float)G_avg, (float)B_avg, H_avg, S_avg, V_avg);
        if (H_avg > 0.2 && H_avg < 0.4)
            ROS_INFO("GREEN");
        else if (H_avg > 0.4 && H_avg < 0.7)
            ROS_INFO("BLUE");
        else if (H_avg < 0.2 || H_avg > 0.8)
            ROS_INFO("RED");
        else
            ROS_INFO("OTHER COLOR");
        ROS_INFO("H_avg: %f, S_avg: %f, V_avg: %f", H_avg, S_avg, V_avg);
        /*
        if (R_avg > G_avg && R_avg > B_avg)
            //ROS_INFO("RED");
        else if (G_avg > R_avg && G_avg > B_avg)
            //ROS_INFO("GREEN");
        else if (B_avg > G_avg && B_avg > R_avg)
            //ROS_INFO("BLUE");*/
        //ROS_INFO("R_avg: %lf G_avg: %lf B_avg: %lf R_acc: %lf G_acc: %lf B_acc: %lf cloud_size: %d", R_avg, G_avg, B_avg, R_acc, G_acc, B_acc, size);
    }

    static void RGB2HSV(float r, float g, float b, float &h, float &s, float &v)
    {
        float K = 0.f;

        if (g < b)
        {
            std::swap(g, b);
            K = -1.f;
        }

        if (r < g)
        {
            std::swap(r, g);
            K = -2.f / 6.f - K;
        }

        float chroma = r - std::min(g, b);
        h = fabs(K + (g - b) / (6.f * chroma + 1e-20f));
        s = chroma / (r + 1e-20f);
        v = r/255;
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
