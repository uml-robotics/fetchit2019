#pragma once

#include <actionlib/client/simple_action_client.h>
#include <fetchit_challenge/SchunkMachineAction.h>

class SchunkMachine{
public:
    SchunkMachine();
    void open();
    void close();
private:
    actionlib::SimpleActionClient<fetchit_challenge::SchunkMachineAction> c;
};
