#include <tf/LinearMath/Quaternion.h>
#include <tf/tf.h>

struct MapPose {
    double x, y, theta;

    bool empty() {
        return x == 0 && y == 0 && theta == 0;
    }

    friend std::ostream& operator<<(std::ostream& s, const MapPose & p) {
        s << "MapPose(" << p.x << ", " << p.y << ", "<< p.theta << ")";
    }

    geometry_msgs::PoseStamped to_pose_stamped_msg() {
        auto frame = "map";

        tf::Quaternion quat;
        quat.setRPY(0.0, 0.0, theta);
        tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), frame);
        geometry_msgs::PoseStamped goal;
        tf::poseStampedTFToMsg(p, goal);
        ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f", frame,
                 goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                 goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, theta);

        return goal;
    }
};
