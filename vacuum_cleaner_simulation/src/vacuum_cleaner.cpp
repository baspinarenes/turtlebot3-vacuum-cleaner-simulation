/*
Enes Başpınar / 06.01.2021
*/

#include "vacuum_cleaner.h"
#include "geometry_msgs/Twist.h"
#include <ros/time.h>

const static double FORWARD_SPEED = 0.2;    // Lineer hız
const static double ROTATE_SPEED = 0.5;     // Açısal hız
const static double MIN_SCAN_ANGLE = -30.0; // Ön taraffın taranacağı alan min açısı (deg)
const static double MAX_SCAN_ANGLE = +30.0; // Ön taraffın taranacağı alan max açısı (deg)
// Durmak için 60 derece bir görüş açısındaki engelleri takip ediyor olacağız
const static float MIN_DIST_FROM_OBSTACLE = 0.4; // Engellerin algılanacağı mesafe (m)

std::vector<float> scan_ranges;

VacuumCleaner::VacuumCleaner()
{
    keepMoving = true;
    keepRotating = false;
    velocityPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // Robotun hızını değiştirebilmek için cmd_vel topic'ine yayın yapıyoruz
    laserSubscriber = node.subscribe("scan", 10, &VacuumCleaner::scanCallback, this);
    // Lazer verileri için ise scan topic'e üye oluyoruz
}

void VacuumCleaner::moveForward()
{
    /*
    Robotu ilerletecek hızı verir.
    */

    stopAngularSpeed();
    geometry_msgs::Twist msg;
    msg.linear.x = FORWARD_SPEED;
    velocityPublisher.publish(msg);
}

void VacuumCleaner::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    /*
    Robotun lazer yardımıyla ölçeceği verileri döndürür.
    scan->ranges içerisinde indeks değerine eşit olan (0-359) açılardaki mesafeleri döndürür.
    Lidar mesafesinde engel yoksa inf döndürür.
    */

    bool isObstacleInFront = false;

    scan_ranges = scan->ranges;

    int minIndex = 360 + MIN_SCAN_ANGLE; // Örnekte -30 derece olduğundan sonuç 330 derece olur.
    int maxIndex = MAX_SCAN_ANGLE; // Bu değer ise örnekte 30 derecedir.

    for (int i = 0; i < maxIndex; i++) // 30 kere döndüreceğiz
    {
        // İlk if'te 330-360 arasındaki uzaklıkları kontrol ediyoruz.
        if (scan->ranges[minIndex + i] < MIN_DIST_FROM_OBSTACLE)
        {
            isObstacleInFront = true;
            break;
        }

        // İlk if'te 0-30 arasındaki uzaklıkları kontrol ediyoruz.
        if (scan->ranges[i] < MIN_DIST_FROM_OBSTACLE)
        {
            isObstacleInFront = true;
            break;
        }
    }

    // Eğer 60 derecelik görüş açımızda engel varsa dönme işlemine başlar.
    if (isObstacleInFront)
    {
        keepRotating = true;
    }
    else
    {
        keepRotating = false;
    }
}

void VacuumCleaner::startMoving()
{
     /*
    Robotu döngüye sokarak sürekli çalışmasını sağlar.
    */

    ros::Rate rate(10);
    ROS_INFO("Start moving");

    while (ros::ok() && keepMoving)
    {
        // Engel görüp görmemesine bağlı olarak yoluna devam eder veyahut dönme işlemine geçer.
        if (keepRotating)
        {
            std::cout << "Dönülüyor" << std::endl;
            rotate();
        }
        else
        {
            std::cout << "İlerleniyor" << std::endl;
            moveForward();
        }

        ros::spinOnce();
        rate.sleep();
    }
}

void VacuumCleaner::stopLinearSpeed()
{
    /*
    Lineer hızı sıfırlayarak ilerlemeyi durdurur.
    */

    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    velocityPublisher.publish(msg);
}

void VacuumCleaner::stopAngularSpeed()
{
    /*
    Açısal hızı sıfırlayarak dönmeyi durdurur.
    */

    geometry_msgs::Twist msg;
    msg.angular.z = 0;
    velocityPublisher.publish(msg);
}

int VacuumCleaner::findMaxDistanceAngle()
{
    /*
    Sağ ve soldaki 60 derecelik görüş açısında (sol için 60-120, sağ için 240-300) 
    bulunan en uzak engelin açısını elde eder.
    */

    double max_distance = 0;
    double deg_of_max_distance = 0;

    for (int j = MIN_SCAN_ANGLE; j < MAX_SCAN_ANGLE; j++)
    {
        if (!isinf(scan_ranges[90 + j]) && scan_ranges[90 + j] > max_distance)
        {
            max_distance = scan_ranges[90 + j];
            deg_of_max_distance = 90 + j;
        }

        if (!isinf(scan_ranges[270 + j]) && scan_ranges[270 + j] > max_distance)
        {
            max_distance = scan_ranges[270 + j];
            deg_of_max_distance = 270 + j;
        }
    }

    return deg_of_max_distance;
}

void VacuumCleaner::rotate()
{
    /*
    Robotun dönmesini sağlar.
    */

    stopLinearSpeed();

    int deg_of_max_distance = findMaxDistanceAngle();

    double rad_of_max_distance = deg_of_max_distance / 180.0 * M_PI;
    std::cout << "Açı: " << deg_of_max_distance << std::endl;

    geometry_msgs::Twist msg;
    msg.angular.z = abs(ROTATE_SPEED);

    // Bu kısımda süre tutmak için timer başlatılır. Ardından dönüş açısına göre sürekli olarak
    // o anki açı hesaplanır ve açı bizimkinden büyük veya eşit olduğunda ise dönüldüğü anlaşılır.
    
    double t0 = ros::Time::now().toSec();
    double current_angle = 0.0;
    ros::Rate loop_rate(1000);
    do
    {
        velocityPublisher.publish(msg);
        double t1 = ros::Time::now().toSec();
        current_angle = ROTATE_SPEED * (t1 - t0);
        ros::spinOnce();
        loop_rate.sleep();

    } while (current_angle < rad_of_max_distance);

    msg.angular.z = 0;
    velocityPublisher.publish(msg);

    keepRotating = false;
}
