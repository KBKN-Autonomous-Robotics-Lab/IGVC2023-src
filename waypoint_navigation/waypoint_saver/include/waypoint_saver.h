#include <string>
#include <sstream>

class Waypoint {
public:
    double x, y, z;
    float vel;
    float rad;
    bool stop;

    Waypoint() : 
        x(0.0),
        y(0.0),
        z(0.0),
        vel(1.0),
        rad(0.8),
        stop(false)
    {
    }


    std::string getYAMLstr()
    {
        std::stringstream ss;
        ss << "{";
        ss << "x: "    << x    << ", " ;
        ss << "y: "    << y    << ", " ;
        ss << "z: "    << z    << ", " ;
        ss << "vel: "  << vel  << ", " ;
        ss << "rad: "  << rad  << ", " ;
        ss << "stop: " << (stop ? "true" : "false") << "}" ;
        return ss.str();
        // {x: *, y: *, z: *, vel: *, rad: *, stop: *}
    }

};