#pragma once

#include <string>

class PlacingParts {
public:
    PlacingParts() {};

    void place_bolt();
    void place_large_gear();
    void place_small_gear();
    void place_gearbox();

private:
    void place(const std::string &what);
};