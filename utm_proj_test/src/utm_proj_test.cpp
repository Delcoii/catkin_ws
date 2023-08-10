#include <ros/ros.h>


#include "lanelet2_core/LaneletMap.h"
#include "lanelet2_projection/UTM.h"

int main(int argc, char** argv) {

    ros::init (argc, argv, "utm_projection_node");

    // Define the WGS84 latitude and longitude coordinates
    double lat = -0.0006044;
    double lon = -0.0001415;
    double ref_lat = 0.;
    double ref_lon = 0.;

    // Define the origin of the UTM projection
    lanelet::Origin origin({ref_lat, ref_lon});

    // Create a UTM projector
    auto projector = lanelet::projection::UtmProjector(origin);

    // Convert the WGS84 coordinates to UTM coordinates
    lanelet::BasicPoint3d utm_coords = projector.forward({lat, lon, 0});

    // Print the UTM coordinates
    std::cout << "UTM Coordinates: " << utm_coords.x() << ", " << utm_coords.y() << std::endl;

    return 0;
}

