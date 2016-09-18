#include <ros/ros.h>
#include <iostream>
#include <cmath>

#include <std_msgs/Float32.h>

using namespace std;

ros::Publisher leftWheel;
ros::Publisher rightWheel;

void wheelSolver(float steeringRotation) {
    // Distance between the steering linkage and the steering column
    const float rs = 2.69258;

    // Angle between the two linkage pivots on the steering column
    const float wheelAngle = 0.380506377*2;

    // Distance between the wheel pivot point and the linkage pivot point
    const float w = 4.0;

    // Length of the steer-wheel linkage
    const float l = 7.0;

    // Position of the wheel pivots relative to the steering column
    const float wlx = -8.0, wly = 1.5;
    const float wrx = 8.0, wry = 1.5;
    
    float theta = steeringRotation + 3.14159;

    float alpha = wheelAngle / 2.0;

    float slx = sin(theta + alpha) * rs;
    float sly = cos(theta + alpha) * rs;

    float srx = sin(theta - alpha) * rs;
    float sry = cos(theta - alpha) * rs;

    // Location of the wheel pivot point relative to the steering linkage on the side of interest
    float wx[2] = {wlx - slx, wrx - srx};
    float wy[2] = {wly - sly, wry - sry};

    // Output array
    float angle[2];

    for(int i = 0; i < 2; i++) {
        // Named constant to simplify calculations
        float D = -0.5 * (w*w - l*l - wx[i]*wx[i] - wy[i]*wy[i]);

        // Quadratic equation constants
        float A = -wx[i]*wx[i] - wy[i]*wy[i];
        float B = 2*D*wy[i];
        float C = -D*D + wx[i]*wx[i] * l*l;

        float y0 = (-B + sqrt(B*B - 4*A*C)) / (2*A);

        float E = (wy[i] - y0) / w;

        if(abs(E - 1.0) < 0.00001) E = 1.0;
        angle[i] = acos(E);
    }
    std_msgs::Float32 msg;

    msg.data = angle[0] * 180/3.14159;
    leftWheel.publish(msg);

    msg.data = angle[1] * 180/3.14159;
    rightWheel.publish(msg);
}

void updateWheels(const std_msgs::Float32::ConstPtr& msg)
{
    wheelSolver(msg->data * (3.14159 / 180.0));
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "forward_solver");
    ros::NodeHandle n;

    leftWheel = n.advertise<std_msgs::Float32>("left_wheel_angle", 1000);
    rightWheel = n.advertise<std_msgs::Float32>("right_wheel_angle", 1000);

    ros::Subscriber steering_angle = n.subscribe("steering_angle", 1000, updateWheels);

    ros::spin();
}

