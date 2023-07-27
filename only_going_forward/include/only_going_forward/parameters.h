#ifndef __PARAM__
#define __PARAM__

#include <yaml-cpp/yaml.h>
#include <string>

class param {
public:

    param();

    double LOOP_RATE;

    double WHEEL_BASE_HALF_M;

    double THROTTLE_MAX;
    double THROTTLE_MIN;

    double P_GAIN;
    double I_GAIN;
    double D_GAIN;
    double TARGET_VELOCITY_MS;

    double LOOKAHEAD_DIST_M;
    double STANLEY_CONSTANT;
    double MIN_ARCTAN_NUMERATOR;

    double CYBERTRUCK_MAX_STEER_DEG;
    double CYBERTRUCK_MIN_STEER_DEG;

};


param::param() {
    std::string stanley_yaml_file_loc = "/home/delcoii/catkin_ws/src/only_going_forward/config/stanley_param.yaml";
    YAML::Node stanley_param_node = YAML::LoadFile(stanley_yaml_file_loc);

    
    LOOP_RATE = stanley_param_node["LOOP_RATE"].as<double>();

    WHEEL_BASE_HALF_M = stanley_param_node["WHEEL_BASE_HALF_M"].as<double>();

    THROTTLE_MAX = stanley_param_node["THROTTLE_MAX"].as<double>();
    THROTTLE_MIN = stanley_param_node["THROTTLE_MIN"].as<double>();

    P_GAIN = stanley_param_node["P_GAIN"].as<double>();
    I_GAIN = stanley_param_node["I_GAIN"].as<double>();
    D_GAIN = stanley_param_node["D_GAIN"].as<double>();
    TARGET_VELOCITY_MS = stanley_param_node["TARGET_VELOCITY_MS"].as<double>();

    LOOKAHEAD_DIST_M = stanley_param_node["LOOKAHEAD_DIST_M"].as<double>();
    STANLEY_CONSTANT = stanley_param_node["STANLEY_CONSTANT"].as<double>();
    MIN_ARCTAN_NUMERATOR = stanley_param_node["MIN_ARCTAN_NUMERATOR"].as<double>();

    CYBERTRUCK_MAX_STEER_DEG = stanley_param_node["CYBERTRUCK_MAX_STEERING_DEG"].as<double>();
    CYBERTRUCK_MIN_STEER_DEG = stanley_param_node["CYBERTRUCK_MIN_STEERING_DEG"].as<double>();
    
    /*
    LOOP_RATE = stanley_param_node["LOOP_RATE"];

    WHEEL_BASE_HALF_M = stanley_param_node["WHEEL_BASE_HALF_M"];

    THROTTLE_MAX = stanley_param_node["THROTTLE_MAX"];
    THROTTLE_MIN = stanley_param_node["THROTTLE_MIN"];

    P_GAIN = stanley_param_node["P_GAIN"];
    I_GAIN = stanley_param_node["I_GAIN"];
    D_GAIN = stanley_param_node["D_GAIN"];
    TARGET_VELOCITY_MS = stanley_param_node["TARGET_VELOCITY_MS"];

    LOOKAHEAD_DIST_M = stanley_param_node["LOOKAHEAD_DISTANCE"];
    STANLEY_CONSTANT = stanley_param_node["STANLEY_CONSTANT"];
    MIN_ARCTAN_NUMERATOR = stanley_param_node["MIN_ARCTAN_NUMERATOR"];

    CYBERTRUCK_MAX_STEER_DEG = stanley_param_node["CYBERTRUCK_MAX_STEERING_DEG"];
    CYBERTRUCK_MIN_STEER_DEG = stanley_param_node["CYBERTRUCK_MIN_STEERING_DEG"];
    */
}

#endif // __PARAM__