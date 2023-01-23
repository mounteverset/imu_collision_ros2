from rclpy.node import Node
import rclpy
import math
import numpy

from sensor_msgs.msg import Imu
#from collision_msgs.msg import Collision
from std_msgs.msg import Bool   

_EPS = numpy.finfo(float).eps * 4.0
# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]
# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}
_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())

class CollisionDetector(Node):
    def __init__(self):
        super().__init__('collision_detector')
        #self.publisher_ = self.create_publisher(Collision, 'collision', 10)
        self.collision_pub = self.create_publisher(Bool, "collision", 10)
        self.rollover_pub = self.create_publisher(Bool, "rollover", 10)
        self.subscriber = self.create_subscription(Imu, "imu_broadcaster/imu", self.imu_callback, 10)

        #self.collision_msg = Collision()
        self.collision_msg = Bool()
        self.rollover_msg = Bool()
        self.imu_msg = Imu()

        self.impact_acceleration = 0.5 #m/s^2 (earth gravity 0.98)
        self.impact_pitch = 0.05 #rads
        self.previous_pitch = 0   
        self.debug = False
        self.get_logger().info("Init complete")
        print("Init complete")
    
    def imu_callback(self, msg):
        acc_z = msg.linear_acceleration.z
        acc_x = msg.linear_acceleration.x
        self.collision_msg.data = False
        self.rollover_msg.data = False

        quaternion = (
        msg.orientation.w,
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z)

        euler = self.euler_from_quaternion(quaternion)

        roll = euler[0]
        pitch = euler[1] #forward-backwards
        yaw = euler[2]

        if abs(acc_x) > self.impact_acceleration:
            #print("ACC crash")
            pitch_change = abs(pitch-self.previous_pitch)
            if pitch_change>self.impact_pitch:
                #print("PITCH crash")
                self.collision_msg.data = True
        if acc_z < 0:
            self.rollover_msg.data = True

        self.previous_pitch = pitch
        
        self.collision_pub.publish(self.collision_msg)
        self.rollover_pub.publish(self.rollover_msg)

    # def detector(self):
    #     pass

    def euler_from_matrix(self, matrix, axes='sxyz'):
        try:
            firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
        except (AttributeError, KeyError):
            _TUPLE2AXES[axes]  # validation
            firstaxis, parity, repetition, frame = axes
        i = firstaxis
        j = _NEXT_AXIS[i+parity]
        k = _NEXT_AXIS[i-parity+1]
        M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:3, :3]
        if repetition:
            sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
            if sy > _EPS:
               az = math.atan2( M[j, i], -M[k, i])
            else:"collision_detector"
        else:
            cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
            if cy > _EPS:
                ax = math.atan2( M[k, j],  M[k, k])
                ay = math.atan2(-M[k, i],  cy)
                az = math.atan2( M[j, i],  M[i, i])
            else:
                ax = math.atan2(-M[j, k],  M[j, j])
                ay = math.atan2(-M[k, i],  cy)
                az = 0.0
        if parity:
            ax, ay, az = -ax, -ay, -az
        if frame:
            ax, az = az, ax
        return ax, ay, az

    def quaternion_matrix(self, quaternion):
        q = numpy.array(quaternion, dtype=numpy.float64, copy=True)
        n = numpy.dot(q, q)
        if n < _EPS:
            return numpy.identity(4)
        q *= math.sqrt(2.0 / n)
        q = numpy.outer(q, q)
        return numpy.array([
            [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
            [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
            [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
            [                0.0,                 0.0,                 0.0, 1.0]])

    def euler_from_quaternion(self, quaternion, axes='sxyz'):
        return self.euler_from_matrix(self.quaternion_matrix(quaternion), axes)


def main(args=None):
    rclpy.init(args=args)
    collision_detector = CollisionDetector()

    rclpy.spin(collision_detector)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
