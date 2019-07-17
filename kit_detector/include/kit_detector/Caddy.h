#pragma once

class CaddyModel;

class Caddy {
public:
    constexpr static double width = 0.23495;
    constexpr static double width_4 = width / 4.0;
    constexpr static double height = 0.12835;
    constexpr static double handle_height = 0.08905 - 0.045704;
    constexpr static double handle_bar_height = 0.012082;
    constexpr static double handle_connector_height = 0.02;
    constexpr static double handle_width = 0.015;
};

class CaddyModel {
public:
    constexpr static double top_z = 0.08905;
};
