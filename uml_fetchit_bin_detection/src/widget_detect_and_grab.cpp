#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include <fetch_cpp/PointHeadClient.h>
#include <fetch_cpp/FetchGripper.h>

#include <tf/transform_broadcaster.h>

#include <caddy_manipulation/CaddyPicking.h>
#include <caddy_manipulation/CaddyGrasping.h>

#include "caddy_manipulation/move_group_util.h"
#include <moveit/move_group_interface/move_group_interface.h>

#include "caddy_manipulation/MoveFetch.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "pcl_ros/transforms.h"
#include "pcl/common/pca.h"
#include "pcl/common/common.h"
#include "pcl/common/geometry.h"
#include "caddy_manipulation/OctomapBuilder.h"
#include <tf_conversions/tf_eigen.h>
#include "control_msgs/GripperCommandActionGoal.h"
#include "std_srvs/SetBool.h"
#include "math.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"



using moveit::planning_interface::MoveGroupInterface;


struct grasp_position {
    pcl::PointXYZ center;
    tf::Transform transform;
    double length;
};

//grasp_position grasp_positions;

std::vector<grasp_position> bottom_grasp_positions, top_grasp_positions, screw_grasp_positions, small_gear_grasp_positions, large_gear_horizontal_grasp_positions, large_gear_vertical_grasp_positions;

std::vector<grasp_position> _bottom_grasp_positions, _top_grasp_positions, _screw_grasp_positions, _small_gear_grasp_positions, _large_gear_horizontal_grasp_positions, _large_gear_vertical_grasp_positions;


ros::Publisher filtered_pub, objects_pub, bin_pub, screw_pub, gear_pub;

ros::Publisher vis_pub;

ros::Publisher gripper_pub;

pcl::PointXYZ center_top, center_bottom;

ros::Publisher cmd_pub;
ros::Subscriber laser_sub;



MoveFetch* move_fetch;
tf::TransformListener *listener;
OctomapBuilder *octomap_builder;
FetchGripper *gripper;
tf::TransformBroadcaster *tf_broadcaster;

float small_gear_max_length = 0.005f, small_gear_min_length = 0.001f;

float big_gear_max_length = 0.025f, big_gear_min_length = 0.010f;

float screw_max_length = 0.0065f, screw_min_length = 0.001f;

float gearbox_max_length = 0.043f, gearbox_min_length = 0.015f;

double small_gear_length = 0.0045; //4.5cm
double large_gear_length = 0.012; //12cm

double gear_differential = ((large_gear_length + small_gear_length) / 3.0) / 1.0; //

double gearbox_bottom_height = .045; //4.5cm
double gearbox_top_height = 0.025;// 2.5cm
double gearbox_differential = (gearbox_bottom_height + gearbox_top_height) / 2.0; //.035


bool screw_sensing_is_enabled= false, gearbox_sensing_is_enabled = false, small_gear_sensing_is_enabled = false;
bool big_gear_sensing_is_enabled = false;
bool process_pointclouds = false;

bool playing_from_bag = false;


//tf::Transform tf_frame;


tf::StampedTransform lookup_transform(const std::string& target_frame, const std::string& source_frame) {
    tf::StampedTransform transform;

    tf::TransformListener tf_listener;
    try {
        tf_listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(10.0) );
        tf_listener.lookupTransform (target_frame, source_frame, ros::Time(0), transform);
    } catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
    }

    return transform;
}

void transformPointCloud(const std::string &target_frame, const pcl::PointCloud<pcl::PointXYZ> &in,
                         pcl::PointCloud<pcl::PointXYZ> &out) {
    pcl_ros::transformPointCloud(in, out, lookup_transform(target_frame, in.header.frame_id));
}


void convert_to_grasp_pose(geometry_msgs::PoseStamped &handle_center, geometry_msgs::PoseStamped &grasp_pose) {

    tf::Transform handle_center_t;
    tf::poseMsgToTF(handle_center.pose, handle_center_t);

    tf::Transform grasp_t;
    grasp_t.setRotation(handle_center_t.getRotation() * tf::Quaternion(tf::Vector3(1, 0, 0), M_PI_2) * tf::Quaternion(tf::Vector3(0, 0, 1), -M_PI_2) ); //  tf::createQuaternionFromRPY(0,0.17,0)
    grasp_t.setOrigin(handle_center_t.getOrigin() + tf::Vector3(0, 0, FetchGripper::wrist_roll_gripper_z_diff));
    tf_broadcaster->sendTransform(tf::StampedTransform(grasp_t, ros::Time::now(), "base_link", "CommandedPosition"));

    grasp_pose = handle_center;
    tf::poseTFToMsg(grasp_t, grasp_pose.pose);
}

void go_home()
{
    std::vector<std::string> joints = {"torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                                       "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"};
    std::vector<double> values = {0.36, 1.57, 0, 0.0, -1.7, 0.0, -0.57, 0.0};

    if(!playing_from_bag)
        move_group_util::set_joints_target_and_execute(*move_fetch, joints, values);
}

std::vector<grasp_position> getLeftMostGrasp(std::vector<grasp_position> grasps)
{
    for(int i = 0; i < grasps.size(); i++)
    {
        //grasps[i].center.y;
    }
}

bool computeBoundingBox(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud)
{
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*transformed_cloud, minPoint, maxPoint);

    float d = pcl::geometry::squaredDistance(maxPoint, minPoint);
    float max_length, min_length;

    if(screw_sensing_is_enabled)
    {
        max_length = screw_max_length;
        min_length = screw_min_length;
    }
    else if(gearbox_sensing_is_enabled)
    {
        max_length = gearbox_max_length;
        min_length = gearbox_min_length;
    }
    else if(small_gear_sensing_is_enabled)
    {
        max_length = small_gear_max_length;
        min_length = small_gear_min_length;
    }
    else if(big_gear_sensing_is_enabled)
    {
        max_length = big_gear_max_length;
        min_length = big_gear_min_length;
    }

    if(d < min_length || d > max_length)
    {
        ROS_INFO("Expected Cluster to be between [%f] and [%f] but this one is [%f]",min_length, max_length, d);
        return false;
    }

   // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //cloud->header.frame_id = "head_camera_rgb_optical_frame";
    //transformPointCloud("base_link", *cloud, *transformed_cloud);


    pcl::PointXYZ center, between_highest_and_center;
    pcl::PointXYZ  highest_point = pcl::PointXYZ();
    pcl::PointXYZ lowest_point = pcl::PointXYZ();

    pcl::CentroidPoint<pcl::PointXYZ> centroid;


    float maxHeight = -1000.0;
    float minHeight = 1000.0;

    for(pcl::PointCloud<pcl::PointXYZ>::iterator itt = transformed_cloud->begin(); itt != transformed_cloud->end(); itt++)
    {
        pcl::PointXYZ tmpPoint(itt->x,itt->y,itt->z);
        centroid.add(tmpPoint);

        if(itt->z > maxHeight)
        {
            maxHeight = itt->z;
            highest_point.x = itt->x;
            highest_point.y = itt->y;
            highest_point.z = itt->z;
        }
        if(itt-> z < minHeight)
        {
            minHeight = itt->z;
            lowest_point.x = itt->x;
            lowest_point.y = itt->y;
            lowest_point.z = itt->z;
        }
    }

    //ROS_INFO("MAXIMUM HEIGHT OF THIS CLISTER IS %f", maxHeight);

    centroid.get(center);

    pcl::CentroidPoint<pcl::PointXYZ> midpoint;

    midpoint.add(highest_point);
    midpoint.add(center);

    centroid.get(between_highest_and_center);



    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(transformed_cloud);
    pca.setIndices(0,0,1,transformed_cloud->size());

    Eigen::RowVector3f major_vector = pca.getEigenVectors().col(0);

    pcl::PointXYZ minpoint, maxpoint;
    pcl::getMinMax3D(*transformed_cloud, minpoint, maxpoint);

    double x_mid_point = (minpoint.x + maxpoint.x)/2.0;
    double y_mid_point = (minpoint.y + maxpoint.y)/2.0;
    double z_mid_point = (minpoint.z + maxpoint.z)/2.0;



    //;//z value

    Eigen::Quaternionf eigen_quaternionf2 = Eigen::Quaternionf::FromTwoVectors( Eigen::Vector3f::UnitY(), major_vector);
    tf::Quaternion tf_quaternion2;
    tf::quaternionEigenToTF(eigen_quaternionf2.cast<double>(), tf_quaternion2);
    tf::Transform tf_frame2 =  tf::Transform(tf_quaternion2, tf::Vector3(0, 0, 0));

    double roll, pitch, yaw;
    tf::Matrix3x3(tf_frame2.getRotation()).getRPY(roll, pitch, yaw);

    //ROS_INFO("INITIAL Roll:[%f]     Pitch:[%f]     Yaw:[%f]", roll, pitch, yaw);



    major_vector(2) = 0;
    Eigen::Quaternionf eigen_quaternionf = Eigen::Quaternionf::FromTwoVectors( Eigen::Vector3f::UnitY(), major_vector);
    tf::Quaternion tf_quaternion;
    tf::quaternionEigenToTF(eigen_quaternionf.cast<double>(), tf_quaternion);

    //double pitch_offset = 0.025;

    double pitch_offset = 0.0;//-1 * (highest_point.y - center.y);

    double roll_offset = 0.0; // 0.035;//

    roll = -0.3;

    bool left_or_right = false, back_or_forward =false;
    if(maxpoint.y - minpoint.y > 0.08)
    {
        left_or_right = true;
        ROS_ERROR("PROBABLY LEFT OR RIGHT");
        ROS_ERROR("MIN X: [%f] Y: [%f]", minpoint.x, minpoint.y);
        ROS_ERROR("MAX X: [%f] Y: [%f]", maxpoint.x, maxpoint.y);
    }

     if(maxpoint.x - minpoint.x > 0.08 ) //distance away from robot, max of .012
    {
        back_or_forward = true;
        ROS_ERROR("PROBABLY AWAY FROM US");
        ROS_ERROR("MIN X: [%f] Y: [%f]", minpoint.x, minpoint.y);
        ROS_ERROR("MAX X: [%f] Y: [%f]", maxpoint.x, maxpoint.y);

    }
    else
    {
        ROS_ERROR("I DON'T KNOW WHICH WAY THIS IS FACING");
    }


    if(left_or_right && (highest_point.y - center.y) > 0.0)
    {
        //pitch_offset = 0.025;
        ROS_ERROR("GEAR IS FACING RIGHT???? [%f] ", (highest_point.y - center.y));
        yaw += M_PI;
        roll = -0.3;
        //pitch_offset = 0.01;

        //yaw += 0.3;

        //roll *= -1;
        //roll = -0.3;
    }
    else if(left_or_right && (highest_point.y - center.y) < 0.0)
    {
        roll = -0.3;
        //roll = 0.3;
        pitch_offset = 0.02;
        ROS_ERROR("GEAR IS FACING LEFT???? [%f] ", (highest_point.y - center.y));
    }

    //else
        if(back_or_forward && (highest_point.x - center.x) > 0.02)//FLAWLESS
    {
        ROS_ERROR("GEAR IS FACING TOWARDS ME???? [%f] ", (highest_point.x - center.x));
        //roll = -0.3;
        yaw += M_PI;
        roll_offset = -0.025;//35;
	//pitch_offset = -0.01;
    }
    else if(back_or_forward)//((highest_point.x - center.x) < -0.01) //FLAWLESS
    {
       // roll = -0.3;
        roll_offset = 0.035;
        //pitch_offset = 0.01;
        ROS_ERROR("GEAR IS FACING AWAY???? [%f] ", (highest_point.x - center.x));
    }

    if(yaw > 2*M_PI)
    {
        yaw-=M_PI;
    }

    tf::Transform tf_frame;

    if( d > gear_differential && big_gear_sensing_is_enabled )
    {

        if(roll > M_PI_2)
        {
            roll = roll - M_PI;
            ROS_ERROR("ROLL IS TOO HIGH, ADJUSTING");
        }
        else if(roll < -1*M_PI_2)
        {
            roll = M_PI + roll;
            ROS_ERROR("ROLL IS TOO LOW, ADJUSTING");
        }

        pitch = 0.0;

        tf_frame =  tf::Transform(tf_quaternion, tf::Vector3(x_mid_point, y_mid_point, center.z));

        tf_frame.setRotation( /*tf_frame.getRotation() */ tf::createQuaternionFromRPY( roll ,pitch,yaw)); //-1 *
        tf_frame.setOrigin(tf_frame.getOrigin() + tf::Vector3(roll_offset, pitch_offset,0));
        //tf_frame.setRotation( tf_frame.getRotation() * tf::createQuaternionFromRPY(0,0,0));

        tf_broadcaster->sendTransform(tf::StampedTransform(tf_frame, ros::Time::now(), "base_link", "DetectedObject"));
        double roll2, pitch2, yaw2;
        tf::Matrix3x3(tf_frame.getRotation()).getRPY(roll2, pitch2, yaw2);

        //ROS_INFO("Final Roll:[%f]     Pitch:[%f]     Yaw:[%f]", roll2, pitch2, yaw2);
    }
    else
    {
        tf_frame =  tf::Transform(tf_quaternion, tf::Vector3(center.x, center.y, center.z));
    }



    tf_broadcaster->sendTransform(tf::StampedTransform(tf::Transform(tf_frame.getRotation() , tf::Vector3(highest_point.x,highest_point.y,highest_point.z)), ros::Time::now(), "base_link", "HighestPoint"));

    tf_broadcaster->sendTransform(tf::StampedTransform(tf::Transform(tf_frame.getRotation() , tf::Vector3(center.x,center.y,center.z)), ros::Time::now(), "base_link", "CenterPoint"));



    //pcl::PointXYZ farthest_from_highest;
    /*Eigen::Vector4f highest_eigen4f = highest_point.getVector4fMap();//Eigen::Vector4f(highest_point);
    Eigen::Vector4f farthest_from_highest;
    pcl::getMaxDistance(*transformed_cloud, farthest_from_highest, farthest_from_highest);*/


    grasp_position pos;
    pos.center = center;
    pos.transform = tf_frame;
    pos.length = d;

    //if(gear_sensing_is_enabled || screw_sensing_is_enabled)
    //    pos.center = highest_point;

    if(gearbox_sensing_is_enabled) {
        double height_diff = maxHeight-minHeight;
        ROS_DEBUG_STREAM("The Height Differential is" << height_diff);
        if(height_diff < 0.1) {
            if (height_diff > gearbox_differential) {
                ROS_INFO("GOT A BOTTOM BOX OVER HERE");
                _bottom_grasp_positions.push_back(pos);
            } else {
                ROS_INFO("GOT A TOP BOX OVER HERE");
                _top_grasp_positions.push_back(pos);
            }
        }
        else
        {
            ROS_DEBUG("GEARBOX IS WAY TOO TALL %f", height_diff);
        }
    }
    else if(big_gear_sensing_is_enabled)
    {
        double height_diff = maxHeight-minHeight;
        ROS_DEBUG_STREAM("The Length of this object is " << d);
        ROS_DEBUG_STREAM("The Height of this object is " << height_diff);
        if(d > gear_differential)
        {
            if(height_diff > 0.05) //0.045
            {
                ROS_INFO("GOT A LARGE GEAR Standing Up");
                _large_gear_vertical_grasp_positions.push_back(pos);
            }
            else
            {
                ROS_INFO("GOT A LARGE GEAR Laying Down");
                _large_gear_horizontal_grasp_positions.push_back(pos);
            }
        }
        else
        {
            ROS_ERROR("THIS REALLY SHOULDNT HAPPEN WTF");

        }

    }
    else if(small_gear_sensing_is_enabled)
    {
        ROS_INFO("GOT A SMALL GEAR OVER HERE");
        ROS_DEBUG_STREAM("The Length of this object is " << d);
        _small_gear_grasp_positions.push_back(pos);
    }
    else{
        ROS_INFO("GOT A SCREW OVER HERE");
        ROS_DEBUG_STREAM("The Length of this object is " << d);
        _screw_grasp_positions.push_back(pos);
    }
    return true;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr passthrough(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std_msgs::Header hdr, bool publish=true)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.0, 0.9);
    if(small_gear_sensing_is_enabled || big_gear_sensing_is_enabled)
	    pass.setFilterLimits (0.0, 0.82);
    pass.setFilterLimitsNegative(false);
    pass.filter (*cloud_filtered);

    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.75, 1.03);
    pass.setFilterLimitsNegative(false);
    pass.filter(*cloud_filtered2);

    if(publish)
    {
        sensor_msgs::PointCloud2 filtered_msg;
        pcl::toROSMsg(*cloud_filtered2, filtered_msg);
        filtered_msg.header = hdr;
        filtered_msg.header.frame_id = "base_link";
        filtered_pub.publish(filtered_msg);
    }
    return cloud_filtered2;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std_msgs::Header hdr, bool publish=true)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr remove_table( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std_msgs::Header hdr, bool publish=true)
{
    double DISTANCE_THRESHOLD = 0.01;
    if( big_gear_sensing_is_enabled)
        DISTANCE_THRESHOLD = 0.02;

    if(small_gear_sensing_is_enabled)
        DISTANCE_THRESHOLD = 0.01;
    if(screw_sensing_is_enabled)
        DISTANCE_THRESHOLD = 0.02;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted_table (new pcl::PointCloud<pcl::PointXYZ>); //Just the table
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed_table (new pcl::PointCloud<pcl::PointXYZ>); //Everything but the table


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (DISTANCE_THRESHOLD);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) cloud->points.size ();
    // While 30% of the original cloud is still there
    while (cloud->points.size () > 0.3 * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_extracted_table);
        //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_removed_table);
        cloud.swap(cloud_removed_table);
        i++;
    }

    if(publish)
    {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*cloud, msg);
        msg.header = hdr;
        msg.header.frame_id = "base_link";
        gear_pub.publish(msg);
    }

    return cloud;
}

void clear_memory()
{
    top_grasp_positions.clear();
    bottom_grasp_positions.clear();
    screw_grasp_positions.clear();
    small_gear_grasp_positions.clear();
    large_gear_vertical_grasp_positions.clear();
    large_gear_horizontal_grasp_positions.clear();
}

bool sortByX(const grasp_position &lhs, const grasp_position &rhs)
{
    return lhs.transform.getOrigin().x() < rhs.transform.getOrigin().x() ;
}

//gets each cluster
pcl::PointCloud<pcl::PointXYZ>::Ptr last_good_cloud (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> get_clusters(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std_msgs::Header hdr, bool publish=true)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    double cluster_tolerence = 0.02;
    double minimum_cluster_size = 10;
    double maximum_cluster_size = 300;

    if(small_gear_sensing_is_enabled)
    {
        cluster_tolerence = 0.03;
        minimum_cluster_size = 3;
        maximum_cluster_size = 30;
    }

    if(big_gear_sensing_is_enabled)
    {
        cluster_tolerence = 0.03;
        minimum_cluster_size = 20;
    }

    if(gearbox_sensing_is_enabled)
    {
        cluster_tolerence = 0.02;
        minimum_cluster_size = 50;
    }


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tolerence); // 2cm
    ec.setMinClusterSize (minimum_cluster_size);//10
    ec.setMaxClusterSize (maximum_cluster_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    sensor_msgs::PointCloud2 output;

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> list_of_clusters;

    _top_grasp_positions.clear();
    _bottom_grasp_positions.clear();
    _screw_grasp_positions.clear();
    _small_gear_grasp_positions.clear();
    _large_gear_vertical_grasp_positions.clear();
    _large_gear_horizontal_grasp_positions.clear();


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
       // ROS_INFO("Checking Valid Cluster....");

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cloud_cluster->header.frame_id = "base_link";

        if(computeBoundingBox(cloud_cluster))
        {
            //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            list_of_clusters.push_back(cloud_cluster); // push_back(cloud_cluster);
        } else{
            std::cout << "BAD DOG " << cloud_cluster->points.size () << std::endl;
            //list_of_clusters.push_back(cloud_cluster); // push_back(cloud_cluster);

        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_objects (new pcl::PointCloud<pcl::PointXYZ>);

    ROS_DEBUG_STREAM("I GOT %d CLUSTERS" << list_of_clusters.size());
    if(list_of_clusters.size() == 0)
        return list_of_clusters;

    for(int i =0; i < list_of_clusters.size(); i++)
    {
        for(int j = 0; j < list_of_clusters[i]->size(); j++)
        {
            clustered_objects->push_back(list_of_clusters[i]->points[j]);
        }
    }

    bool updated_cloud = false;
    if(_top_grasp_positions.size() > top_grasp_positions.size())
    {
        top_grasp_positions = _top_grasp_positions;
        if(publish)
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*clustered_objects, msg);
            msg.header = hdr;
            msg.header.frame_id = "base_link";
            objects_pub.publish(msg);
            last_good_cloud = clustered_objects;
            updated_cloud = true;
        }
        std::sort(top_grasp_positions.begin(), top_grasp_positions.end(),sortByX);
    }

    if(_bottom_grasp_positions.size() > bottom_grasp_positions.size())
    {
        bottom_grasp_positions = _bottom_grasp_positions;
        if(publish)
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*clustered_objects, msg);
            msg.header = hdr;
            msg.header.frame_id = "base_link";
            objects_pub.publish(msg);
            last_good_cloud = clustered_objects;
            updated_cloud = true;
        }
        std::sort(bottom_grasp_positions.begin(), bottom_grasp_positions.end(),sortByX);
    }

    if(_screw_grasp_positions.size() > screw_grasp_positions.size())
    {
        screw_grasp_positions = _screw_grasp_positions;
        if(publish)
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*clustered_objects, msg);
            msg.header = hdr;
            msg.header.frame_id = "base_link";
            objects_pub.publish(msg);
            last_good_cloud = clustered_objects;
            updated_cloud = true;
        }
        std::sort(screw_grasp_positions.begin(), screw_grasp_positions.end(),sortByX);
    }

    if(_large_gear_vertical_grasp_positions.size() > large_gear_vertical_grasp_positions.size())
    {
        large_gear_vertical_grasp_positions = _large_gear_vertical_grasp_positions;
        if(publish)
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*clustered_objects, msg);
            msg.header = hdr;
            msg.header.frame_id = "base_link";
            objects_pub.publish(msg);
            last_good_cloud = clustered_objects;
            updated_cloud = true;
        }
        std::sort(large_gear_vertical_grasp_positions.begin(), large_gear_vertical_grasp_positions.end(),sortByX);
    }

    if(_large_gear_horizontal_grasp_positions.size() > large_gear_horizontal_grasp_positions.size())
    {
        large_gear_horizontal_grasp_positions = _large_gear_horizontal_grasp_positions;
        if(publish)
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*clustered_objects, msg);
            msg.header = hdr;
            msg.header.frame_id = "base_link";
            objects_pub.publish(msg);
            last_good_cloud = clustered_objects;
            updated_cloud = true;
        }
        std::sort(large_gear_horizontal_grasp_positions.begin(), large_gear_horizontal_grasp_positions.end(),sortByX);
    }

    if(_small_gear_grasp_positions.size() > small_gear_grasp_positions.size())
    {
        small_gear_grasp_positions = _small_gear_grasp_positions;
        if(publish)
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*clustered_objects, msg);
            msg.header = hdr;
            msg.header.frame_id = "base_link";
            objects_pub.publish(msg);
            last_good_cloud = clustered_objects;
            updated_cloud = true;
        }
        std::sort(small_gear_grasp_positions.begin(), small_gear_grasp_positions.end(),sortByX);
    }
    if (!updated_cloud && publish) {
        sensor_msgs::PointCloud2 msg;
        pcl::toROSMsg(*last_good_cloud, msg);
        msg.header = hdr;
        msg.header.frame_id = "base_link";
        objects_pub.publish(msg);
    }
    return list_of_clusters;
}


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
     if(!process_pointclouds)
         return;


  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZ>); //converted from ros msg


  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);//downsampled with voxelfilter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);//downsampled with voxelfilter

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_extracted_table (new pcl::PointCloud<pcl::PointXYZ>); //Just the table
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_removed_table (new pcl::PointCloud<pcl::PointXYZ>); //Everything but the table

  pcl::fromROSMsg(*cloud_msg, *raw_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    transformed_cloud->header.frame_id = "base_link";
    transformPointCloud("base_link", *raw_cloud, *transformed_cloud);


  cloud_filtered  = voxel_filter(transformed_cloud, cloud_msg->header, false); //downsample the cloud

  cloud_filtered2 = passthrough(cloud_filtered, cloud_msg->header,true); //remove points too far away


   cloud_removed_table = remove_table(cloud_filtered2, cloud_msg->header, true);

   std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = get_clusters(cloud_removed_table, cloud_msg->header, true);
}



bool enable_screw_sensing(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    clear_memory();
    if(!playing_from_bag)
    {
        PointHeadClient headClient;
        headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8) );
    }
    process_pointclouds = true;
    screw_sensing_is_enabled = true;
    gearbox_sensing_is_enabled = false;
    small_gear_sensing_is_enabled = false;
    big_gear_sensing_is_enabled = false;
    return res.success = true;
}

bool enable_small_gear_sensing(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    clear_memory();
    if(!playing_from_bag)
    {
        PointHeadClient headClient;
        headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8) );
    }
    process_pointclouds = true;
    small_gear_sensing_is_enabled = true;
    big_gear_sensing_is_enabled = false;
    screw_sensing_is_enabled = false;
    gearbox_sensing_is_enabled = false;
    return res.success = true;
}

bool enable_big_gear_sensing(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    clear_memory();
    if(!playing_from_bag)
    {
        PointHeadClient headClient;
        headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8) );
    }
    process_pointclouds = true;
    small_gear_sensing_is_enabled = false;
    big_gear_sensing_is_enabled = true;
    screw_sensing_is_enabled = false;
    gearbox_sensing_is_enabled = false;
    return res.success = true;
}

bool enable_gearbox_sensing(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    clear_memory();
    if(!playing_from_bag)
    {
        PointHeadClient headClient;
        headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8) );
    }

    process_pointclouds = true;
    gearbox_sensing_is_enabled = true;
    screw_sensing_is_enabled = false;
    small_gear_sensing_is_enabled = false;
    big_gear_sensing_is_enabled = false;
    return res.success = true;
}

bool disable_sensing(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    process_pointclouds = false;
    return res.success = true;
}

bool grasp_box( tf::Transform _tf)
{
    geometry_msgs::PoseStamped grasp_pose, gear_center, gear_center_fixed;
    gear_center.header.frame_id="base_link";

    gear_center.pose.position.x = _tf.getOrigin().x();
    gear_center.pose.position.y = _tf.getOrigin().y();
    gear_center.pose.position.z = _tf.getOrigin().z();

    //tf::Quaternion quat = tf::createQuaternionFromRPY(90.0, 0.0, 0.0);
    //_tf.setRotation(_tf.getRotation() *= quat);

    gear_center.pose.orientation.x = _tf.getRotation().x();
    gear_center.pose.orientation.y = _tf.getRotation().y();
    gear_center.pose.orientation.z = _tf.getRotation().z();
    gear_center.pose.orientation.w = _tf.getRotation().w();

    convert_to_grasp_pose(gear_center, grasp_pose);


    if(!playing_from_bag)
    {
        octomap_builder->clear_map();

        PointHeadClient headClient;

        std::vector<Eigen::Vector3d> poses;

        poses.push_back(Eigen::Vector3d(0.7, 0 , 0.0));

        poses.push_back(Eigen::Vector3d(0.7, -0.5, 0.0 ));

        poses.push_back(Eigen::Vector3d(0.7, 0.5, 0.0 ));

        poses.push_back(Eigen::Vector3d(0.7, 0.5, -0.8 ));

        poses.push_back(Eigen::Vector3d(0.7, -0.5,-0.8 ));



        octomap_builder->relatively_look_around_to_build_map(poses);


        headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8) );

        double gripper_pos = 0.08;

        if(screw_sensing_is_enabled)
            gripper_pos = 0.04;


        bool plan_only = false;
        bool success = true;



        CaddyGrasping grasping(*move_fetch, gear_center, grasp_pose);
        if (!grasping.approach(plan_only, gripper_pos, .2))
        {
            ROS_ERROR("Grasping failed on first approach, returning");
            return false;
        };

        if (!grasping.approach(plan_only, gripper_pos, .05) && success)
        {
            ROS_ERROR("Grasping failed on second approach, returning");
            grasping.approach(plan_only, gripper_pos, .2);
            return false;
        };

        octomap_builder->clear_map();

        if(!grasping.grasp(plan_only) && success)
        {
            ROS_ERROR("Grasping failed on grasp, returning");
            grasping.approach(plan_only, gripper_pos, .05);
            grasping.approach(plan_only, gripper_pos, .2);
            return false;
        }

        if(big_gear_sensing_is_enabled)
        {
            ros::Duration(1.0).sleep();
            if(gripper->getPosition() < 0.026)
            {
                ROS_ERROR("GOT A BAD GRIP ON A GEAR, BAILING");
                gripper->open(gripper_pos);
                success = false;
                grasping.approach(plan_only, gripper_pos, .05);
                grasping.approach(plan_only, gripper_pos, .2);
                return false;
            }
            //0.028889264911413193 good
            //0.01232602447271347 bad
        }

        if(!grasping.lift_up(plan_only, 0.05) && success)
        {
            ROS_ERROR("Grasping failed on First lift, returning");
            return false;
        }


        if(!grasping.lift_up(plan_only, 0.2))
        {
            ROS_ERROR("Grasping failed on second lift, returning");
            return false;
        }


        auto laser_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/base_scan");
        geometry_msgs::Twist twist;
        twist.angular.z = 0.0;

        ros::Rate loop_rate(10);

        twist.linear.x = -0.1;

        while( ros::ok() )//&& laser_scan->ranges[laser_scan->ranges.size() / 2] > 0.1)
        {
            laser_scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/base_scan");
            double range;
            for(int j = 330; j < 350; j++)
            {
                if(laser_scan->ranges[j] > 0)
                    range = laser_scan->ranges[j];
            }

            if(range > 0.6)
                break;
            //ROS_INFO_STREAM("MOVING CLOSER " << range); //662
            cmd_pub.publish(twist);
            ros::spinOnce();
            loop_rate.sleep();
        }


        if(gripper->getPosition() < 0.005)
        {
            ROS_ERROR("I should have an object here but I don't.");
            return false;
        }
    }
    return true;
}

bool grasp_box_top(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    bool success;
    if(!top_grasp_positions.empty())
    {
        success = grasp_box(top_grasp_positions.front().transform);
        top_grasp_positions.erase(top_grasp_positions.begin());
    }
    else
    {
        success = false;
        ROS_ERROR("Failed to detect a gearbox top piece");
    }

    go_home();
    return res.success = success;
}

bool grasp_box_bottom(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    bool success;
    if(!bottom_grasp_positions.empty())
    {
        success = grasp_box(bottom_grasp_positions.back().transform);
        bottom_grasp_positions.pop_back();
    }
    else
    {
        success = false;
        ROS_ERROR("Failed to detect a gearbox bottom piece");
    }

    go_home();
    return res.success = success;
}

bool grasp_screw(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    bool success;
    if(!screw_grasp_positions.empty())
    {
        success = grasp_box( screw_grasp_positions[screw_grasp_positions.size() / 2].transform);
        screw_grasp_positions.erase(screw_grasp_positions.begin());
    }
    else
    {
        success = false;
        ROS_ERROR("Failed to detect a screw piece");
    }

    go_home();
    return res.success = success;
}

bool grasp_small_gear(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    bool success;
    if(!small_gear_grasp_positions.empty())
    {
        success = grasp_box( small_gear_grasp_positions.front().transform);
        small_gear_grasp_positions.erase(small_gear_grasp_positions.begin());
    }
    else
    {
        success = false;
        ROS_ERROR("Failed to detect a  small gear");
    }

    go_home();
    return res.success = success;
}

bool grasp_large_gear(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    bool success;
    if(!large_gear_horizontal_grasp_positions.empty() )
    {
        success = grasp_box( large_gear_horizontal_grasp_positions.back().transform);
        large_gear_horizontal_grasp_positions.pop_back();
    }
    else if( false && !large_gear_vertical_grasp_positions.empty() )
    {
        success = grasp_box(large_gear_vertical_grasp_positions.front().transform);
        large_gear_vertical_grasp_positions.erase(large_gear_vertical_grasp_positions.begin());
    }
    else
    {
        success = false;
        ROS_ERROR("Failed to detect a large gear");
    }
    go_home();
    return res.success = success;
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "GrabbingGearBoxes");
    ros::NodeHandle nh;

    octomap_builder = new OctomapBuilder(nh);
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ROS_INFO_STREAM("move_group: wait_for_servers...");
    if(!playing_from_bag)
        move_fetch = new MoveFetch();
    gripper = new FetchGripper();
    listener = new tf::TransformListener();
    tf_broadcaster = new tf::TransformBroadcaster();

    filtered_pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 1);
    objects_pub = nh.advertise<sensor_msgs::PointCloud2> ("/objects_cloud", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker> ("/visualization_marker", 1);
    gear_pub = nh.advertise<sensor_msgs::PointCloud2> ("/gear_cloud", 1);


    cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 5);


    ros::Subscriber sub = nh.subscribe ("/head_camera/depth_downsample/points", 1, cloud_cb);



    ros::ServiceServer small_gear_sensing_service = nh.advertiseService("/small_gear/enable_sensing",enable_small_gear_sensing);
    ros::ServiceServer large_gear_sensing_service = nh.advertiseService("/large_gear/enable_sensing",enable_big_gear_sensing);
    ros::ServiceServer screw_sensing_service = nh.advertiseService("/screw/enable_sensing",enable_screw_sensing);
    ros::ServiceServer bottom_box_sensing_service = nh.advertiseService("/bottom_box/enable_sensing",enable_gearbox_sensing);
    ros::ServiceServer top_box_sensing_service = nh.advertiseService("/top_box/enable_sensing",enable_gearbox_sensing);

    ros::ServiceServer small_gear_grasp_service = nh.advertiseService("/small_gear/grasp",grasp_small_gear);
    ros::ServiceServer large_gear_grasp_service = nh.advertiseService("/large_gear/grasp",grasp_large_gear);
    ros::ServiceServer screw_grasp_service = nh.advertiseService("/screw/grasp",grasp_screw);
    ros::ServiceServer bottom_box_grasp_service = nh.advertiseService("/bottom_box/grasp",grasp_box_bottom);
    ros::ServiceServer top_box_grasp_service = nh.advertiseService("/top_box/grasp",grasp_box_top);

    ros::ServiceServer disable_sensing_service = nh.advertiseService("/disable_sensing",disable_sensing);


    ROS_INFO("Services up and running");

    ros::waitForShutdown();
}
