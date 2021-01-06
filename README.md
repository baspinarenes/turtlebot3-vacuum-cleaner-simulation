# turtlebot3-vacuum-cleaner-simulation

![alt text](https://github.com/baspinarenes/turtlebot3-vacuum-cleaner-simulation/blob/main/ros_gazebo_rviz.png?raw=true)


Projenin amacı, ROS Gazebo simülasyonundaki turtlebot3 robotunun bir odada sürekli olarak gezmesi ve engel gördüğünde en uzak mesafeye yönelerek bu işlemleri tekrar etmesidir. Bu proje, Kocaeli Üniversitesi Bilgisayar Mühendisliği 4. Sınıf Gezgin Robotlara Giriş dersi kapsamında yapılmıştır. 

## Projenin Çalıştırılması

Projeyi indirdikten sonra "vacuum-cleaner-simulation" isimli klasörü "catkin_ws/src" yoluna kopyalayın.

Terminalde "catkin_ws" yoluna giderek projeyi derleyin:

```bash
cd ~/catkin_ws
catkin_make
```

Derledikten sonra launch dosyası ile çalıştırın:

```bash
roslaunch vacuum_cleaner_simulation run_vacuum_cleaner.launch 
```
