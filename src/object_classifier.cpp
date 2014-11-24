#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_msgs/Classification.h>
#include <s8_msgs/DistPose.h>
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

// OPENCV specific includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/synchronizer.h>

// OTHER
#include <vector>
#include <signal.h>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>


// DEFINITIONS
#define HZ                  10
#define BUFFER_SIZE         1

#define NODE_NAME           		"s8_object_classification_node"
#define TOPIC_POINT_CLOUD   		"/s8/detectedObject"
#define TOPIC_RGB_IMAGE             "/camera/rgb/image_rect_color"
#define TOPIC_EXTRACTED_OBJECTS		"/s8/modifiedObject"
#define TOPIC_OBJECT_TYPE           "/s8/Classification/type"
#define TOPIC_RECIEVED_DISTPOSE     "/s8/ip/detection/distPose"
#define TOPIC_SENT_DISTPOSE         "/s8/ip/classification/distPose"
#define CONFIG_DOC                  "/home/ras/catkin_ws/src/s8_ip_classification/parameters/parameters.json"


static const std::string OPENCV_WINDOW = "Image window";

typedef pcl::PointXYZRGB PointT;
using namespace std;
using namespace cv;

class ObjectClassifier : public s8::Node
{
    const int hz;

    ros::Subscriber point_cloud_subscriber;
    ros::Publisher point_cloud_publisher;
    ros::Subscriber rgb_image_subscriber;
    ros::Publisher object_type_publisher;
    ros::Subscriber object_distPose_subscriber;
    ros::Publisher object_distPose_publisher;

    pcl::PointCloud<PointT>::Ptr cloud;
    cv_bridge::CvImagePtr rgb_image;
    s8_msgs::DistPose distPose;
    bool cloudInitialized;
    bool rgbImageInitialized;
    bool distPoseInitialized;

    float circle_red_low_H, circle_red_high_H, circle_yellow_low_H, circle_yellow_high_H;
    float cube_red_low_H, cube_red_high_H, cube_yellow_low_H, cube_yellow_high_H;
    float cube_green_low_H, cube_green_high_H, cube_blue_low_H, cube_blue_high_H;
    float others_orange_low_H, others_orange_high_H, others_purple_low_H, others_purple_high_H;
    float others_green_low_H, others_green_high_H, others_blue_low_H, others_blue_high_H;

    int param_plane_count_points;

public:
    ObjectClassifier(int hz) : hz(hz)
    {
        add_params();
        //printParams();
        point_cloud_subscriber = nh.subscribe(TOPIC_POINT_CLOUD, BUFFER_SIZE, &ObjectClassifier::point_cloud_callback, this);
        point_cloud_publisher  = nh.advertise<sensor_msgs::PointCloud2> (TOPIC_EXTRACTED_OBJECTS, BUFFER_SIZE);
        rgb_image_subscriber   = nh.subscribe(TOPIC_RGB_IMAGE, BUFFER_SIZE, &ObjectClassifier::rgb_image_callback, this);
        object_type_publisher  = nh.advertise<s8_msgs::Classification> (TOPIC_OBJECT_TYPE, BUFFER_SIZE);
        object_distPose_subscriber = nh.subscribe(TOPIC_RECIEVED_DISTPOSE, BUFFER_SIZE, &ObjectClassifier::object_distPose_callback, this);
        object_distPose_publisher  = nh.advertise<s8_msgs::DistPose> (TOPIC_SENT_DISTPOSE, BUFFER_SIZE);
        cloud = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud<PointT>);
        cloudInitialized    = false;
        rgbImageInitialized = false;
        distPoseInitialized = false;

        //cv::namedWindow(OPENCV_WINDOW);
    }
    ~ObjectClassifier()
    {
        //cv::destroyWindow(OPENCV_WINDOW);
    }

    void updateClass()
    {
        s8_msgs::Classification classType;
        classType.type = 0;
        classType.name = "No Object";

        if ( cloudInitialized == false || rgbImageInitialized == false)
        {
            ROS_INFO("No OBJECT FOUND");
        }
        else if (isWhite(cloud))
        {
            ROS_INFO("WHITE OBJECT, IGNORED");
        }
        else if (recognizeSphereObject(cloud))
        {
            classType = findCircleClass();
        }
        else if (recognizePlaneObject(cloud))
        {
            classType = findCubeClass();
        }
        else
        {
            classType = findOthersClass();
        }

        if (classType.type == 0)
            distPose.dist = -1;

        cloudPublish(cloud);
        typePublish(classType);
        distPosePublish(distPose);
        cloudInitialized = false;
    }

private:

    bool isWhite(pcl::PointCloud<PointT>::Ptr cloud_white)
    {
        float H = 0.0, S = 0.0, V = 0.0;
        getColors(cloud_white, &H, &S, &V);
        if ((H < 0.1 || H > 0.9) && S < 0.1 && V > 0.9 )
            return true;
        else
            return false;
    }

    s8_msgs::Classification findCircleClass()
    {
        s8_msgs::Classification classType;
        float H = 0.0, S = 0.0, V = 0.0;
        getColors(cloud, &H, &S, &V);
        if(H > circle_red_high_H || H < circle_red_low_H)
        {
            ROS_INFO("Seeing red circle");
            classType.type = 1;
            classType.name = "Red Circle";
        }
        else if (H < circle_yellow_high_H && H > circle_yellow_low_H)
        {
            ROS_INFO("Seeing yellow circle");
            classType.type = 2;
            classType.name = "Yellow Circle";
        }
        /*else
        {
            ROS_INFO("Seeing something else");
            classType.type = 0;
            classType.name = "No Object";
        }*/
        return classType;
    }

    s8_msgs::Classification findCubeClass()
    {
        s8_msgs::Classification classType;
        float H = 0.0, S = 0.0, V = 0.0;
        getColors(cloud, &H, &S, &V);
        if(H > cube_red_high_H || H < cube_red_low_H)
        {
            ROS_INFO("Seeing red cube");
            classType.type = 3;
            classType.name = "Red Cube";
        }
        else if (H < cube_yellow_high_H && H > cube_yellow_low_H)
        {
            ROS_INFO("Seeing yellow cube");
            classType.type = 4;
            classType.name = "Yellow Cube";
        }
        else if (H < cube_green_high_H && H > cube_green_low_H)
        {
            ROS_INFO("Seeing green cube");
            classType.type = 5;
            classType.name = "Green Cube";
        }
        else //(H < cube_blue_high_H && H > cube_blue_low_H)
        {
            ROS_INFO("Seeing blue cube");
            classType.type = 6;
            classType.name = "Blue Cube";
        }
        return classType;
    }

    s8_msgs::Classification findOthersClass()
    {
        s8_msgs::Classification classType;
        float H = 0.0, S = 0.0, V = 0.0;
        getColors(cloud, &H, &S, &V);
        if(H > others_orange_high_H || H < others_orange_low_H)
        {
            ROS_INFO("Seeing orange star");
            classType.type = 7;
            classType.name = "Orange Star";
        }
        else if (H < others_purple_high_H && H > others_purple_low_H)
        {
            ROS_INFO("Seeing purple cross");
            classType.type = 8;
            classType.name = "Purple Cross";
        }
        else if (H < cube_green_high_H && H > cube_green_low_H)
        {
            ROS_INFO("Seeing green cylinder");
            classType.type = 9;
            classType.name = "Green Cylinder";
        }
        else //(H < cube_blue_high_H && H > cube_blue_low_H)
        {
            ROS_INFO("Seeing blue triangle");
            classType.type = 10;
            classType.name = "Blue Triangle";
        }
        return classType;
    }

    void getColors(pcl::PointCloud<PointT>::Ptr cloud_color, float *H, float *S, float *V)
    {
        double R_avg = 0;
        double G_avg = 0;
        double B_avg = 0;
        double R_acc = 0;
        double G_acc = 0;
        double B_acc = 0;
        float H_acc = 0;
        float S_acc = 0;
        float V_acc = 0;
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
        }
        //ROS_INFO("R_low: %d R_high: %d G_low: %d G_high: %d B_low: %d B_high: %d", R_low, R_high, G_low, R_high, B_low, B_high);
        int size = cloud_color->points.size();
        R_avg = R_acc / size;
        G_avg = G_acc / size;
        B_avg = B_acc / size;
        RGB2HSV((float)R_avg, (float)G_avg, (float)B_avg, *H, *S, *V);
        ROS_INFO("H: %f, S: %f, V: %f", *H, *S, *V);
        /*
        if (H_avg > 0.2 && H_avg < 0.4)
            ROS_INFO("GREEN CUBE");
        else if (H_avg > 0.4 && H_avg < 0.7)
            ROS_INFO("BLUE CUBE");
        else if (H_avg < 0.2 || H_avg > 0.7){
            if (H_avg < 0.18 && S_avg > 0.5 && V_avg > 0.4){
                ROS_INFO("ORANGE STAR");}
            else if (H_avg > 0.7 && S_avg > 0.25 && V_avg > 0.4){
            ROS_INFO("PURPLE CROSS");}
            else
            ROS_INFO("RED");}
        else
            ROS_INFO("OTHER COLOR");
        //ROS_INFO("H_avg: %f, S_avg: %f, V_avg: %f", H_avg, S_avg, V_avg);
        /*
        if (R_avg > G_avg && R_avg > B_avg)
            //ROS_INFO("RED");
        else if (G_avg > R_avg && G_avg > B_avg)
            //ROS_INFO("GREEN");
        else if (B_avg > G_avg && B_avg > R_avg)
            //ROS_INFO("BLUE");*/
        //ROS_INFO("R_avg: %lf G_avg: %lf B_avg: %lf R_acc: %lf G_acc: %lf B_acc: %lf cloud_size: %d", R_avg, G_avg, B_avg, R_acc, G_acc, B_acc, size);
    }

    bool recognizePlaneObject(pcl::PointCloud<PointT>::Ptr cloud_seg)
    {
        std::vector<std::vector<float> > coeffMatrix;
        int inlierSize = 0;

        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        pcl::ModelCoefficients coeff;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        pcl::ExtractIndices<PointT> extract;
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setKSearch (50);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.001);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (1000);

        int i = 0, nr_points = (int) cloud_seg->points.size ();
        if (nr_points < 50){
            return false;
        }
        // While 10% of the original cloud is still there
        while (cloud_seg->points.size () > 0.1 * nr_points && i < 3)
        {
            //seg.setInputCloud (cloud);
            ne.setInputCloud (cloud_seg);
            ne.compute (*cloud_normals);
            seg.setInputCloud (cloud_seg);
            seg.setInputNormals (cloud_normals);
            seg.segment (*inliers, coeff);
            if (inliers->indices.size () == 0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }
            if(inliers->indices.size() < nr_points*0.05|| inliers->indices.size() < 20){
                i++;
                continue;
            }
            // Extract the planar inliers from the input cloud
            extract.setInputCloud (cloud_seg);
            extract.setIndices (inliers);
            extract.setNegative (true);
            extract.filter (*cloud_plane);
            cloud_seg.swap (cloud_plane);
            coeffMatrix.push_back(coeff.values);
            i++;
            if (inliers->indices.size() > inlierSize)
                inlierSize = inliers->indices.size();
        }
        if (inlierSize > param_plane_count_points)
        {
            if (coeffMatrix.size() == 2)
            {
                float angle = getAngle(coeffMatrix[0],coeffMatrix[1]);
                if (abs(angle - 90) > 8 )
                {
                    ROS_INFO("2 PLANES");
                    return false;
                }
            }
            else if (coeffMatrix.size() == 3)
            {
                float angle1 = getAngle(coeffMatrix[0],coeffMatrix[1]);
                float angle2 = getAngle(coeffMatrix[0],coeffMatrix[2]);
                float angle3 = getAngle(coeffMatrix[1],coeffMatrix[2]);
                if (!(abs(angle1 - 90) < 10 || abs(angle2 - 90) < 10 || abs(angle3 - 90) < 10))
                {
                    ROS_INFO("3 PLANES");
                    return false;
                }
            }
            else{
                ROS_INFO("1 PLANE");
            }
            return true;
        }
        else
            return false;
    }

    bool recognizeSphereObject(pcl::PointCloud<PointT>::Ptr cloud_seg)
    {
        int inlierSize = 0;

        pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
        pcl::ModelCoefficients coeff;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

        pcl::ExtractIndices<PointT> extract;
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;

        // Estimate point normals
        ne.setSearchMethod (tree);
        ne.setKSearch (50);

        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_SPHERE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.001);
        seg.setNormalDistanceWeight (0.1);
        seg.setMaxIterations (1000);
        seg.setRadiusLimits (0.010, 0.05);

        int i = 0, nr_points = (int) cloud_seg->points.size ();
        if (nr_points < 50){
            return false;
        }

        //seg.setInputCloud (cloud);
        ne.setInputCloud (cloud_seg);
        ne.compute (*cloud_normals);
        seg.setInputCloud (cloud_seg);
        seg.setInputNormals (cloud_normals);
        seg.segment (*inliers, coeff);
        if (inliers->indices.size() > 80)
            return true;
        else
            return false;
    }

    bool isCircle()
    {
        if(rgbImageInitialized == false) return false;

        Mat HSVImage;
        Mat RGBImage;

        // Convert from BRG to HSV and use gaussian blur.
        colorPreProcess(&HSVImage, &RGBImage);

        Mat imgThresholded;

        Mat cimg;
        Mat maskedimg;
        RGBImage.copyTo(maskedimg, imgThresholded);
        medianBlur(maskedimg, maskedimg, 5);

        cvtColor(maskedimg, cimg, COLOR_BGR2GRAY);

        vector<Vec3f> circles;
        HoughCircles(cimg, circles, CV_HOUGH_GRADIENT, 1, 10,
                     100, 30, 1, 30 // change the last two parameters
                                    // (min_radius & max_radius) to detect larger circles
                     );

        for( size_t i = 0; i < circles.size(); i++ )
        {
            Vec3i c = circles[i];
            circle( maskedimg, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, CV_AA);
            circle( maskedimg, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, CV_AA);
        }
        cv::imshow(OPENCV_WINDOW, maskedimg);
        cv::waitKey(3);
        if (circles.size() > 0)
            return true;
        else
            return false;
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

    float getAngle (std::vector<float> v1, std::vector<float> v2)
    {
        float dotproduct    = v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
        float len1          = v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2];
        float len2          = v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2];
        float angleRad      = acos(dotproduct/(sqrt(len1)*sqrt(len2)));
        return angleRad*180/3.1415;
    }

    void colorPreProcess(cv::Mat *HSVImage, cv::Mat *RGBImage)
    {
        cv::Mat tempHSV, tempRGB, frame;
        //Convert the captured frame from BGR to HSV
        cvtColor(rgb_image->image, tempHSV, COLOR_BGR2HSV);
        tempRGB = rgb_image->image;
        *HSVImage = tempHSV;
        *RGBImage = tempRGB;
        GaussianBlur(*RGBImage, frame, cv::Size(0, 0), 4);
        addWeighted(*RGBImage, 1.6, frame, -0.5, 0, *RGBImage);
        //GaussianBlur( *RGBImage, *RGBImage, Size(5, 5), 1, 1 );
    }

    void point_cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        pcl::fromROSMsg (*cloud_msg, *cloud);
        cloudInitialized = true;
    }

    void object_distPose_callback(s8_msgs::DistPose distPoseIn)
    {
        distPose.pose = distPoseIn.pose;
        distPose.dist = distPoseIn.dist;
        distPoseInitialized = true;
    }

    void rgb_image_callback(const sensor_msgs::ImageConstPtr& msgColor)
    {
        try
        {
            rgb_image = cv_bridge::toCvCopy(msgColor, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        rgbImageInitialized = true;
    }

    void cloudPublish(pcl::PointCloud<PointT>::Ptr cloud_pub)
    {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud_pub, output);
        point_cloud_publisher.publish (output);
    }

    void typePublish(s8_msgs::Classification classType)
    {
        object_type_publisher.publish(classType);
    }

    void distPosePublish(s8_msgs::DistPose distPose)
    {
        object_distPose_publisher.publish(distPose);
        ROS_INFO("Distance: %f, Angle: %f", distPose.dist, distPose.pose);
    }

    void add_params()
    {
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(CONFIG_DOC, pt);
        // CIRCLE
        circle_red_low_H = pt.get<float>("circle_red_low_H");
        circle_red_high_H = pt.get<float>("circle_red_high_H");
        circle_yellow_low_H = pt.get<float>("circle_yellow_low_H");
        circle_yellow_high_H = pt.get<float>("circle_yellow_high_H");
        // CUBE
        cube_red_low_H = pt.get<float>("cube_red_low_H");
        cube_red_high_H = pt.get<float>("cube_red_high_H");
        cube_yellow_low_H = pt.get<float>("cube_yellow_low_H");
        cube_yellow_high_H = pt.get<float>("cube_yellow_high_H");
        cube_green_low_H = pt.get<float>("cube_green_low_H");
        cube_green_high_H = pt.get<float>("cube_green_high_H");
        cube_blue_low_H = pt.get<float>("cube_blue_low_H");
        cube_blue_high_H = pt.get<float>("cube_blue_high_H");
        // OTHERS
        others_orange_low_H = pt.get<float>("others_orange_low_H");
        others_orange_high_H = pt.get<float>("others_orange_high_H");
        others_purple_low_H = pt.get<float>("others_purple_low_H");
        others_purple_high_H = pt.get<float>("others_purple_high_H");
        others_green_low_H = pt.get<float>("others_green_low_H");
        others_green_high_H = pt.get<float>("others_green_high_H");
        others_blue_low_H = pt.get<float>("others_blue_low_H");
        others_blue_high_H = pt.get<float>("others_blue_high_H");

        // SETTINGS
        param_plane_count_points = pt.get<int>("param_plane_count_points");
    }
};

int main(int argc, char **argv) {

    std::cout << argv[0] << std::endl;
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
