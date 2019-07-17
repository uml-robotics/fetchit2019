#pragma once

#include <actionlib/client/simple_action_client.h>
#include <fetchit_challenge/SickCameraAction.h>

class SickCamera{
public:
    SickCamera();
    void activate();
private:
    actionlib::SimpleActionClient<fetchit_challenge::SickCameraAction> c;
};
