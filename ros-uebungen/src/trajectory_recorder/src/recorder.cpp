#include <memory>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

// Struct fürs Speichern der berechneten Positionen
struct Position {
    double x;
    double z;
    double theta;
};

class TrajectoryRecorder : public rclcpp::Node
{
public:
    TrajectoryRecorder(std::ofstream& odom_out, std::ofstream& imu_out) :
            Node("trajectory_recorder"), odom_out_(odom_out), imu_out_(imu_out)
    {
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "imu_broadcaster/imu",
                10,
                std::bind(&TrajectoryRecorder::imu_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "rosbot_base_controller/odom",
                10,
                std::bind(&TrajectoryRecorder::odom_callback, this, _1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("trajectory_recorder/imu_pose", 10);

        global_x_ = 0;
        global_y_ = 0;
        global_z_ = 0;
        last_time_ = 0;
        first_ = true;
        first_odom_ = true;
    }


private:

    Position lastCalc = {
            .x = 0,
            .z = 0,
            .theta = 0
    };


    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Zeitstempel der Nachricht in Sekunden umwandeln
        double timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

        // Beim ersten Aufruf, letzten Zeitstempel setzen und Funktion beenden
        if(last_time_ == 0){
            last_time_ = timestamp;
            return;
        }

        // Zeitdifferenz zur letzten Nachricht
        double deltaT = last_time_ - timestamp;

        // Lineare Beschleunigungswerte aus IMU Nachricht
        double x = msg->linear_acceleration.x;
        double y = msg->linear_acceleration.y;
        double z = msg->linear_acceleration.z;

        // Winkelgeschwindigkeit der Z-Achse aus IMU Nachricht
        double anz = msg->angular_velocity.z;

        // Geschwindigkeiten in x, y und z Richtung berechnen
        double vx = x * deltaT;
        double vy = y * deltaT;
        double vz = z * deltaT;

        // Gesamtgeschwindigkeit berechnen
        double v = std::sqrt( std::pow(vx, 2) + std::pow(vy, 2) + std::pow(vz, 2));

        // Änderung des Drehwinkels berechnen
        double deltaTheta = anz * deltaT;

        // Letzte berechnete Position und Winkel extrahieren
        double lastX = this->lastCalc.x;
        double lastZ = this->lastCalc.z;
        double lastTheta = this->lastCalc.theta;

        // Neue Position und Winkel berechnen
        double xn = lastX + (v * deltaT * std::sin(lastTheta + (deltaTheta / 2) * deltaT));
        double zn = lastZ + (v * deltaT * std::cos(lastTheta + (deltaTheta / 2) * deltaT));
        double thetan = lastTheta + (deltaTheta * deltaT);

        // Neue berechnete Werte speichern
        this->lastCalc.x = xn;
        this->lastCalc.z = zn;
        this->lastCalc.theta = thetan;

        // In Datei schreiben
        if(imu_out_.good())
        {
            imu_out_ << xn << " " << zn << std::endl;
        }

        // PoseStamp Nachricht
        geometry_msgs::msg::PoseStamped pose_msg;

        //header
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "base_link";

        //position
        pose_msg.pose.position.x = xn;
        pose_msg.pose.position.y = 0;
        pose_msg.pose.position.z = zn;

        //orientation
        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = std::sin(thetan / 2.0);
        pose_msg.pose.orientation.w = std::cos(thetan / 2.0);

        //publish PoseStamped msg
        pose_pub_->publish(pose_msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry& msg)
    {
        geometry_msgs::msg::PoseWithCovariance pose = msg.pose;
        double x = pose.pose.position.x;
        double y = pose.pose.position.y;
        double z = pose.pose.position.z;

        if(first_odom_)
        {
            first_odom_ = false;
            odom_offset_x_ = pose.pose.position.x;
            odom_offset_y_ = pose.pose.position.y;
            odom_offset_z_ = pose.pose.position.z;
        }
        else
        {
            x = x - odom_offset_x_;
            y = y - odom_offset_y_;
            z = z - odom_offset_z_;
        }

        if(odom_out_.good())
        {
            odom_out_ << x << " " << y << " " << z << std::endl;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::ofstream& odom_out_;
    std::ofstream& imu_out_;

    double global_x_;
    double global_y_;
    double global_z_;

    double odom_offset_x_ = 0.0;
    double odom_offset_y_ = 0.0;
    double odom_offset_z_ = 0.0;

    double last_vel_x_ = 0.0;
    double last_vel_y_ = 0.0;
    double last_vel_z_ = 0.0;

    bool first_;
    bool first_odom_;

    double last_time_ = 0;
};

int main(int argc, char** argv)
{
    std::ofstream odom_out("odometry.txt");
    std::ofstream imu_out("imu.txt");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryRecorder>(odom_out, imu_out));
    rclcpp::shutdown();
}
