#include "ros/ros.h"
#include "cwork3/my_msg.h"
#include "cwork3/lpf.h"
#include "math.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv){

ros::init(argc, argv, "publisher");
ros::NodeHandle nh;

//Create a publisher object:
// Input - type of message: cwork3::my_msg
//       - topic name: /sinu
//       - message queue: 1
ros::Publisher topic_pub;
topic_pub = nh.advertise<cwork3::my_msg>("/sinu", 1);

ros::Rate rate(100);

//Define the custom data type
cwork3::my_msg data;

    cout << "Insert amplitude: " << endl;
    cin >> data.amp;
    cout << "Insert period: " << endl;
    cin >> data.T;

float count = 0;

    while(ros::ok()) {

        data.value = data.amp*sin(2*3.14*(1/data.T)*count);

        ROS_INFO("sin: %.3f\t time: %.2f", data.value, count);

        topic_pub.publish(data);

        count += 1.0/100.0;
        
        //Rate 100Hz
        rate.sleep(); 
    }

return 0;
}