#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState

class Go2LowStateToJoint(Node):
    def __init__(self):
        super().__init__('go2_lowstate_to_joint')
        self.sub = self.create_subscription(LowState, '/lowstate', self.callback, 10)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("✅ Listening /lf/lowstate → Publishing /joint_states")

    def callback(self, msg):
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',    # 왼쪽 앞다리
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',    # 오른쪽 앞다리
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',    # 왼쪽 뒷다리
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',    # 오른쪽 뒷다리
        ]

        # motor_state 배열의 q, dq, tau_est를 JointState로 변환
        # go2_driver.cpp의 매핑을 참조: [3,4,5,0,1,2,9,10,11,6,7,8]
        motor_indices = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
        js.position = [msg.motor_state[i].q for i in motor_indices]
        js.velocity = [msg.motor_state[i].dq for i in motor_indices]
        js.effort   = [msg.motor_state[i].tau_est for i in motor_indices]
        
        self.pub.publish(js)
        

def main(args=None):
    rclpy.init(args=args)
    node = Go2LowStateToJoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
