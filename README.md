#IMU collision detector

This package reads data from 'imu9250' topic and publishes in 'imu9250_collision' topic the following message. 
```bash
std_msgs/Bool collision
std_msgs/Bool rollover
```

Collision detection force is set to 0.7 m/s^2 (being earth gravity 0.98 m/s^2). The value can be modified changing the following parameter in /scripts/collision.py .
```python
impact_acceleration = 0.7
```

Vehicle overturning is also detected, this functionality can be avoided removing this two lines from /scripts/collision.py:
```python
if acc_z < 0:
    collision = "rollover"
```

Posible values:

'true' : collision detected

'false' : no collision detected

'rollover' : the sensor is upside down, the vehicle overturned

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

