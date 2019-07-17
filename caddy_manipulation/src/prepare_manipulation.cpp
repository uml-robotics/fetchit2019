#include <fetch_cpp/TorsoLiftClient.h>
#include <fetch_cpp/PointHeadClient.h>

void prepare_caddy_manipulation () {
    // raise torso
    TorsoLiftClient torsoLiftClient;
    torsoLiftClient.lift_to_highest();

    // look at
    PointHeadClient headClient;
    headClient.relatively_look_at(0.7, 0, -(1.4245 - 0.8));
}