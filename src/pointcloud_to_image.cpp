#include "ros/ros.h"
#include "pointcloud_to_image/pointcloud_to_img_config.h"
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <image_transport/image_transport.h>
#include <pcl/filters/filter.h>
#include <message_filters/sync_policies/approximate_time.h>

image_transport::Publisher pub;
unsigned int height = 1080;
unsigned int width = 1080;

float rad = 50.0; // close(<), far (>)
float pol = 0.0;  // down (+), up  (-)
float azm = 0.0;  // right(+), left(-)

Eigen::MatrixXf Mc(3,4); // camera calibration matrix
Eigen::MatrixXf RT(4,4); // translation matrix lidar-camera

pcl::PointCloud<pcl::PointXYZ>::Ptr saved_pointCloud (new pcl::PointCloud<pcl::PointXYZ>);

void calc_matrices(){
    
    Mc <<  1000.0 ,   0.0 ,height/2 ,0.0
          ,   0.0 ,1000.0 , width/2 ,0.0
          ,   0.0 ,   0.0 ,     1.0 ,0.0;

    RT <<   cos(pol)*cos(azm) ,-cos(pol)*sin(azm) ,sin(pol) ,0.0
          ,          sin(azm) ,          cos(azm) ,     0.0 ,0.0
          ,-sin(pol)*cos(azm) , sin(pol)*sin(azm) ,cos(pol) ,rad
          ,               0.0 ,               0.0 ,     0.0 ,1.0;
}

void proyect_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud){
    
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

    int sizeLidar = (int) pointCloud->points.size();
    Eigen::MatrixXf pointCloud_matrix_image(3,sizeLidar);
    Eigen::MatrixXf pointCloud_matrix_world(4,sizeLidar);

 
    pcl::PointXYZ minV(FLT_MAX, FLT_MAX, FLT_MAX), maxV(FLT_MIN, FLT_MIN, FLT_MIN);
 
    for (int i = 0; i < sizeLidar; i++){
        pointCloud_matrix_world(0,i) = -pointCloud->points[i].x;
        pointCloud_matrix_world(1,i) = -pointCloud->points[i].y;
        pointCloud_matrix_world(2,i) = pointCloud->points[i].z;
        pointCloud_matrix_world(3,i) = 1.0;
 
        if(pointCloud->points[i].x<minV.x) minV.x=pointCloud->points[i].x;
        if(pointCloud->points[i].z<minV.z) minV.z=pointCloud->points[i].z;
        if(pointCloud->points[i].x>maxV.x) maxV.x=pointCloud->points[i].x;
        if(pointCloud->points[i].z>maxV.z) maxV.z=pointCloud->points[i].z;
 
    }

    maxV.x = maxV.x>abs(minV.x)?maxV.x:abs(minV.x);
    minV.x = 0;
    
    pointCloud_matrix_image = Mc * (RT * pointCloud_matrix_world);

    int pu = 0;
    int pv = 0;

 
    for (int i=0;i<sizeLidar;i++){
        pu = (int)(pointCloud_matrix_image(1,i)/pointCloud_matrix_image(2,i));
        pv = (int)(pointCloud_matrix_image(0,i)/pointCloud_matrix_image(2,i));
        
        int color_dis_x = (int)abs(255*((pointCloud->points[i].x-minV.x)/(maxV.x-minV.x)));
        int color_dis_z = (int)(255*((pointCloud->points[i].z-minV.z)/(maxV.z-minV.z)));

        cv::circle(image, cv::Point(pu, pv), 1, CV_RGB(255-color_dis_x,(int)(color_dis_z),color_dis_x),cv::FILLED);
        
    }
    
    // Project axis to image

    Eigen::MatrixXf axis_matrix_image(3,4);
    Eigen::MatrixXf axis_matrix_world(4,4);
    
    axis_matrix_world <<   0.0 ,-1.0 , 0.0 , 0.0
                          ,0.0 , 0.0 ,-1.0 , 0.0
                          ,0.0 , 0.0 , 0.0 , 1.0
                          ,1.0 , 1.0 , 1.0 , 1.0;
    
    axis_matrix_image = Mc * (RT * axis_matrix_world);

    cv::Point axis_c((int)(axis_matrix_image(1,0)/axis_matrix_image(2,0)), (int)(axis_matrix_image(0,0)/axis_matrix_image(2,0)));
    cv::Point axis_x((int)(axis_matrix_image(1,1)/axis_matrix_image(2,1)), (int)(axis_matrix_image(0,1)/axis_matrix_image(2,1)));
    cv::Point axis_y((int)(axis_matrix_image(1,2)/axis_matrix_image(2,2)), (int)(axis_matrix_image(0,2)/axis_matrix_image(2,2)));
    cv::Point axis_z((int)(axis_matrix_image(1,3)/axis_matrix_image(2,3)), (int)(axis_matrix_image(0,3)/axis_matrix_image(2,3)));

    cv::line(image, axis_c, axis_x, CV_RGB(200, 50, 50), 3);
    cv::line(image, axis_c, axis_y, CV_RGB(  0,255,  0), 3);
    cv::line(image, axis_c, axis_z, CV_RGB(  0,  0,255), 3);
    
    // Publish image

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    pub.publish(msg);

    //ROS_INFO("Pointcloud image published");
}

void config_callback (const pointcloud_to_image::pointcloud_to_img_config::ConstPtr& config_msg){
    
    height = config_msg->height;
    width = config_msg->width;
    rad = config_msg->radial;
    pol = -config_msg->polar;
    azm = -config_msg->azimuth;

    calc_matrices();

    proyect_pointcloud(saved_pointCloud);

    //ROS_INFO("Config setted");
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    // Conversion from sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*cloud_msg,pcl_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr msg_pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc,*msg_pointCloud);
    
    // Filter point cloud
    if (msg_pointCloud == NULL) return; 

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pointCloud->header.frame_id = "velodyne";
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*msg_pointCloud, *pointCloud, indices);

    // Save pointcloud in case it config is changed
    saved_pointCloud = pointCloud;

    // Project point cloud to image
    proyect_pointcloud(saved_pointCloud);
}

int main (int argc, char** argv) {
    
    ros::init (argc, argv, "cloud_sub");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    ros::Rate loop_rate(10);
    
    calc_matrices();

    ros::Subscriber sub_pointCloud;
    ros::Subscriber sub_spherCoord;
    
    pub = it.advertise("/pointCloud_image", 1);
    pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv::Mat (height, width, CV_8UC3, cv::Scalar(0, 0, 0))).toImageMsg());
    
    sub_spherCoord = nh.subscribe ("/pointCloud_config", 1, config_callback);
    sub_pointCloud = nh.subscribe ("/velodyne_points", 1, cloud_callback);
    
    ros::spin();
}