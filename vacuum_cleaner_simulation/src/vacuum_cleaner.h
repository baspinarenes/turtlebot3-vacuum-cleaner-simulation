#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class VacuumCleaner {
public:
    VacuumCleaner();
    void startMoving(); // Robotun kesintisiz şekilde odada hareket etmesini sağlar

private:
    ros::NodeHandle node;
    ros::Publisher velocityPublisher; // Robotun lineer ve açısal hızlarının değiştirilmesini sağlar
    ros::Subscriber laserSubscriber; // Robotun yayınladığı lazer verisi topiğine erişmemizi sağlar
    bool keepMoving; // Robotun sürekli hareket etmesini sağlar
    bool keepRotating;  // Robotun ne zaman döneceğini kontrol eder

    void moveForward(); // Robotu ilerletir
    void rotate(); // Robotu döndürür
    int findMaxDistanceAngle(); // En uzaktaki noktanın açısını elde eder
    void stopLinearSpeed();  // İlerleme hızı sıfırlar
    void stopAngularSpeed(); // Dönme hızını sıfırlar
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan); // Lidar verilerini anlık olarak döndürür
};
