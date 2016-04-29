#IMU collision detector

This package reads data from 'imu9250' topic and publishes in 'imu9250_collision' topic a boolean value as String.
True means collision, False means no collision.

Quick setup:
----- 
```bash
git clone [this repository]
catkin_make_isolated --pkg imu_collision --install

rosrun imu_collision collision.py
```

Running the node:
-----
```bash
rosrun imu_collision collision.py
```

Listening to the topic:
-----
```bash
rostic echo imu9250_collision
```

