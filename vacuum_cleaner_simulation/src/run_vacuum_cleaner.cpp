#include "vacuum_cleaner.h"

int main(int argc, char **argv) {
    // "vacuum_cleaner" isminde ROS düğümü oluşturur
    ros::init(argc, argv, "vacuum_cleaner");

    // Robotun bir nesnesini tanımlar
    VacuumCleaner vacuum_cleaner;

    // Ve harekete geçirir
    vacuum_cleaner.startMoving();

    return 0;
};
