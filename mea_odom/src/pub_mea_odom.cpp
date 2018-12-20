#include "SimpleSerial.h"
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace boost;
SimpleSerial serial("/dev/sensors/keisoku",115200);
std::vector<std::string> split(std::string str, char del) ;

std::vector<std::string> split(std::string str, char del)
 {
    int first = 0;
    int last = str.find_first_of(del);

    std::vector<std::string> result;

    while (first < str.size()) {
        std::string subStr(str, first, last - first);

        result.push_back(subStr);

        first = last + 1;
        last = str.find_first_of(del, first);

        if (last == std::string::npos) {
            last = str.size();
        }
    }

    return result;
}


// 組み込みかの方がいいのではそれとも同期の問題　スライド参照
void mea_odom_callback(double &x ,double &y ,double &th ,double &vx,double &vy,double &omega)
{
 
    std::string str;
    std::vector <std::string> s;
    std::vector <double> d ;
    char del = ',';

    try {
        str = serial.readLine(); 
        
        for (const auto subStr : split(str, del))
        {
            s.push_back(subStr);    
        }
    
        for(int i = 0; i != s.size(); i++)   
        {   
            // d.push_back(std::stod(s[i])) ;
            d.push_back( lexical_cast<double>( s[i] )) ; 
        }

        x = d[0];
        y = d[1];
        th = d[2];
        vx = d[3];
        vy = d[4];
        omega = d[5];

        s.clear();
        d.clear();

    }catch(boost::system::system_error& e)
    {
        cout<<"Error: "<<e.what()<<endl;
        // return 1;
    }


}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odometry_publisher"); //ノード名指定
    ros::NodeHandle nh;
    //  ROS、tfそれぞれにメッセージを送るために、ROSのパブリッシャーとtfの tf::TransformBroadcaster を両方作る必要ある
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time;
    
    // 周期
    ros::Rate r(30.0);
    // 計測輪のオドメトリの初期化t
    double mea_odom_x=0.0;//オドメトリX座標[m]
    double mea_odom_y=0.0;//オドメトリY座標[m]
    double mea_odom_th=0.0; //オドメトリ姿勢[rad]
    double mea_odom_vx=0.0;//オドメトリX座標[m]
    double mea_odom_vy=0.0;//オドメトリY座標[m]
    double mea_odom_omega=0.0; //オドメトリ姿勢[rad]



    while(nh.ok()){

        mea_odom_callback(mea_odom_x,mea_odom_y,mea_odom_th,mea_odom_vx,mea_odom_vy,mea_odom_omega) ;
        current_time = ros::Time::now();
       
        //tf mea_odom->base_link2
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "mea_odom";
        odom_trans.child_frame_id = "base_link2";

        odom_trans.transform.translation.x = mea_odom_x;
        odom_trans.transform.translation.y = mea_odom_y;
        odom_trans.transform.translation.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(mea_odom_th);
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        //  odom?のpose.covarianc

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        // odom.pose.covariance = [1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000000000000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.001];

        odom.header.stamp = current_time;
        odom.header.frame_id = "mea_odom";
        //set the position
        odom.pose.pose.position.x = mea_odom_x;
        odom.pose.pose.position.y = mea_odom_y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        //set the velocity
        odom.child_frame_id = "base_link2";
        odom.twist.twist.linear.x = mea_odom_vx;
        odom.twist.twist.linear.y = mea_odom_vy;
        odom.twist.twist.angular.z =mea_odom_omega; 
        //publish the message
        odom_pub.publish(odom);
        
        
        ros::spinOnce();
        r.sleep();
    }
}

   
