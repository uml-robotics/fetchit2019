#include "ArenaNav.h"

class NavOnlyArena: public ArenaNav {

public:
    NavOnlyArena() : ArenaNav() {}

    void pick_large_gear() override {}
    bool insert_large_gear() override {}
    void close_schunk_machine_door() override {}
    void pick_small_gear() override {}
    void place_small_gear() override {}
    void pick_up_screw() override {}
    void place_screw() override {}
    void pick_up_gearbox_top() override {}
    void pick_up_gearbox_bottom() override {}
    void place_gearbox() override {}
    void open_schunk_machine_door() override {}
    void remove_large_gear() override {}
    void place_machined_large_gear() override {}
    void pick_up_caddy() override {}
    void drop_off_caddy() override {}


    void run() override {
        ROS_INFO_STREAM("Running...");

        ROS_INFO_STREAM("Remembering rotation for caddy " << n_th_caddy << "...");
        CaddyRotationParam first_caddy_rotation_param;
        first_caddy_rotation_param.forget(); // always start clear; may not forget if stop early
        first_caddy_rotation_param.remember();

        std::cout << "Press Enter to go to safe place for arm movement..."; std::cin.ignore();
        go_to_safest_place_for_arm_movement();

        std::cout << "Press Enter to machine large gear..."; std::cin.ignore();
        go_machine_large_gear();

        std::cout << "Press Enter to pick small gear..."; std::cin.ignore();
        go_pick_small_gear();
        std::cout << "Press Enter to place small gear..."; std::cin.ignore();
        go_place_small_gear();

        std::cout << "Press Enter to pick gearbox top..."; std::cin.ignore();
        go_pick_gearbox_top();
        std::cout << "Press Enter to place a gearbox top..."; std::cin.ignore();
        go_place_gearbox();

        std::cout << "Press Enter to pick gearbox bottom..."; std::cin.ignore();
        go_pick_gearbox_bottom();
        std::cout << "Press Enter to place a gearbox bottom..."; std::cin.ignore();
        go_place_gearbox();

        std::cout << "Press Enter to pick screw..."; std::cin.ignore();
        go_pick_screw();
        std::cout << "Press Enter to place screw..."; std::cin.ignore();
        go_place_screw();

        std::cout << "Press Enter to pick machined large gear..."; std::cin.ignore();
        go_pick_machined_large_gear();
        std::cout << "Press Enter to place machined large gear..."; std::cin.ignore();
        go_place_machined_large_gear();

        std::cout << "Press Enter to pick caddy..."; std::cin.ignore();
        go_pick_caddy();
        std::cout << "Press Enter to drop off caddy..."; std::cin.ignore();
        go_drop_off_caddy();

        ROS_INFO_STREAM("Forgetting leftmost caddy rotation...");
        first_caddy_rotation_param.forget();

        ROS_INFO_STREAM("All done!!!");
    }

};
