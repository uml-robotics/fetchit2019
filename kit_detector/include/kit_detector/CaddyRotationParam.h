#pragma once

#include <ros/param.h>
#include <tf/LinearMath/Quaternion.h>

class CaddyRotationParam {
public:

    void remember() {
        ros::param::set(remember_key, "anything");
    }

    bool is_remembering() {
        return ros::param::has(remember_key);
    }

    void forget() {
        ros::param::del(remember_key);
        ros::param::del(rotation_key);
    }

    bool was_set() {
        return ros::param::has(rotation_key);
    }

    tf::Quaternion get_rotation() {
        double x, y, z, w;
        ros::param::get(rotation_key + "/x", x);
        ros::param::get(rotation_key + "/y", y);
        ros::param::get(rotation_key + "/z", z);
        ros::param::get(rotation_key + "/w", w);

        return tf::Quaternion(x, y, z, w);
    }

    void set_rotation(tf::Quaternion quaternion) {
        ros::param::set(rotation_key + "/x", quaternion.x());
        ros::param::set(rotation_key + "/y", quaternion.y());
        ros::param::set(rotation_key + "/z", quaternion.z());
        ros::param::set(rotation_key + "/w", quaternion.w());
    }

private:
    static const std::string rotation_key;
    static const std::string remember_key;
};