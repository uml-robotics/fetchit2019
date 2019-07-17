#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class LargeGearCollisionObject {

public:

    LargeGearCollisionObject();

    void attach();

    void detach_and_remove();

private:
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::AttachedCollisionObject attached_collision_object;

    std::string ID = "large_gear";

    static constexpr double HEIGHT = 0.11; // 0.11518;
    static constexpr double WIDTH = 0.063; // 0.063491;

    void construct_collision_object(const double Z_OFFSET = - (HEIGHT / 2.0));
};
