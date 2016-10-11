#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include <std_msgs/Float64.h>

using namespace std;

ros::Publisher leftWheel;
ros::Publisher rightWheel;

void wheelSolver(double steeringRotation, double *angle) {
    // Distance between the steering linkage and the steering column
    const double rs = 2.69258;

    // Angle between the two linkage pivots on the steering column
    const double wheelAngle = 0.380506377*2;

    // Distance between the wheel pivot point and the linkage pivot point
    const double w = 4.0;

    // Length of the steer-wheel linkage
    const double l = 7.0;

    // Position of the wheel pivots relative to the steering column
    const double wlx = -8.0, wly = 1.5;
    const double wrx = 8.0, wry = 1.5;
    
    double theta = steeringRotation + 3.14159;

    double alpha = wheelAngle / 2.0;

    double slx = sin(theta + alpha) * rs;
    double sly = cos(theta + alpha) * rs;

    double srx = sin(theta - alpha) * rs;
    double sry = cos(theta - alpha) * rs;

    // Location of the wheel pivot point relative to the steering linkage on the side of interest
    double wx[2] = {wlx - slx, wrx - srx};
    double wy[2] = {wly - sly, wry - sry};

    for(int i = 0; i < 2; i++) {
        // Named constant to simplify calculations
        double D = -0.5 * (w*w - l*l - wx[i]*wx[i] - wy[i]*wy[i]);

        // Quadratic equation constants
        double A = -wx[i]*wx[i] - wy[i]*wy[i];
        double B = 2*D*wy[i];
        double C = -D*D + wx[i]*wx[i] * l*l;

        double y0 = (-B + sqrt(B*B - 4*A*C)) / (2*A);

        double E = (wy[i] - y0) / w;

        if(abs(E - 1.0) < 0.00001) E = 1.0;
        angle[i] = acos(E);
    }
}

void updateWheels(const std_msgs::Float64::ConstPtr& msg)
{
    double angle[2];
    wheelSolver(msg->data * (3.14159 / 180.0), angle);

    std_msgs::Float64 msg;

    msg.data = angle[0] * 180/3.14159;
    leftWheel.publish(msg);

    msg.data = angle[1] * 180/3.14159;
    rightWheel.publish(msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "forward_solver");
    ros::NodeHandle n;

    leftWheel = n.advertise<std_msgs::Float64>("left_wheel_angle", 1000);
    rightWheel = n.advertise<std_msgs::Float64>("right_wheel_angle", 1000);

    ros::Subscriber steering_angle = n.subscribe("steering_angle", 1000, updateWheels);

    ros::spin();
}

