import mujoco as mj
import numpy as np
from .mujoco_base import MuJoCoBase
from mujoco.glfw import glfw
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Bool,Int8MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import time
from ros2pkg.api import get_prefix_path
from scipy.spatial.transform import Rotation as R
from threading import Thread
from rclpy.time import Time
import array

init_joint_pos = np.array([0.0, 0.0, -0.37, 0.90, -0.53, 0, 0.0, 0.0, -0.37, 0.90, -0.53, 0])
init_base_pos = np.array([0, 0, 0.955])
init_base_eular_zyx = np.array([0.0, -0., 0.0])
imu_eular_bias = np.array([0.0, 0.0, 0.0])

class HumanoidSim(MuJoCoBase):
  def __init__(self, xml_path = str):

    super().__init__(xml_path)
    
    self.node = Node('hector_sim')
    self.simend = 1000.0
    self.sim_rate = 1000.0
    # print('Total number of DoFs in the model:', self.model.nv)
    # print('Generalized positions:', self.data.qpos)  
    # print('Generalized velocities:', self.data.qvel)
    # print('Actuator forces:', self.data.qfrc_actuator)
    # print('Actoator controls:', self.data.ctrl)
    # mj.set_mjcb_control(self.controller)
    # * Set subscriber and publisher

    # initialize target joint position, velocity, and torque
    self.targetPos = init_joint_pos
    self.targetVel = np.zeros(12)
    self.targetTorque = np.zeros(12)
    self.targetKp = np.zeros(12)
    self.targetKd = np.zeros(12)

    self.pubJoints = self.node.create_publisher(Float32MultiArray, '/jointsPosVel', 2)
    self.pubOdom = self.node.create_publisher(Odometry, '/ground_truth/state', 2)
    self.pubImu = self.node.create_publisher(Imu, '/imu', 2)
    self.pubRealTorque = self.node.create_publisher(Float32MultiArray, '/realTorque', 2)
    self.pubSimState = self.node.create_publisher(Bool,'/pauseFlag', 2)
    # 实测接触标志，长度 4：[l_foot_toe, r_foot_toe, l_foot_heel, r_foot_heel]（同 ModelSettings.h 的 contactNames3DoF）
    self.pubContactFlag = self.node.create_publisher(Int8MultiArray, '/simContactFlag', 2)
    # 缓存上一步接触状态，供 500Hz 发布分支复用（在 1kHz step 中更新）
    self.contactFlag = Int8MultiArray()
    self.contactFlag.data = array.array('b', [0, 0, 0, 0])

    self.node.create_subscription(Float32MultiArray, "/targetTorque", self.targetTorqueCallback,2) 
    self.node.create_subscription(Float32MultiArray, "/targetPos", self.targetPosCallback,2) 
    self.node.create_subscription(Float32MultiArray, "/targetVel", self.targetVelCallback,2)
    self.node.create_subscription(Float32MultiArray, "/targetKp", self.targetKpCallback,2)
    self.node.create_subscription(Float32MultiArray, "/targetKd", self.targetKdCallback,2)
    #set the initial joint position
    self.data.qpos[:3] = init_base_pos
    # init rpy to init quaternion
    # scipy 的 as_quat() 返回 (x,y,z,w)，而 MuJoCo qpos[3:7] 需要 (w,x,y,z)，必须显式转换
    q_xyzw = R.from_euler('xyz', init_base_eular_zyx).as_quat()
    self.data.qpos[3:7] = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
    self.data.qpos[-12:] = init_joint_pos

    self.data.qvel[:3] = np.array([0, 0, 0])
    self.data.qvel[-12:] = np.zeros(12)

    # * show the model
    mj.mj_step(self.model, self.data)
    # enable contact force visualization
    self.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        self.window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
    # Update scene and render
    mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                        mj.mjtCatBit.mjCAT_ALL.value, self.scene)
    mj.mjr_render(viewport, self.scene, self.context)
    

    

  def updateContactFlag(self):
    # 判定两只脚的 sole geom（left_sole / right_sole）是否与地面接触。
    # 顺序同 contactNames3DoF：[l_foot_toe, r_foot_toe, l_foot_heel, r_foot_heel]。
    # 说明：toe 与 heel 两个 site 位于同一个 sole box geom 上，而 MuJoCo 每对 geom 只生成一个接触点，
    # 因此无法据单个接触点区分 toe/heel（压力中心偏移会让 heel 标志误报）。
    # 这里按"整只脚是否触地"判定，同一只脚的 toe/heel 置为相同值——这是模型能可靠观测的物理量。
    ground_gid = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, 'ground')
    sole_gid = {
      'l': mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, 'left_sole'),
      'r': mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_GEOM, 'right_sole'),
    }

    touched = {'l': False, 'r': False}
    for c in range(self.data.ncon):
      con = self.data.contact[c]
      for side in ('l', 'r'):
        g = sole_gid[side]
        if (con.geom1 == g and con.geom2 == ground_gid) or (con.geom2 == g and con.geom1 == ground_gid):
          touched[side] = True

    # [l_foot_toe, r_foot_toe, l_foot_heel, r_foot_heel]
    self.contactFlag.data[0] = 1 if touched['l'] else 0
    self.contactFlag.data[1] = 1 if touched['r'] else 0
    self.contactFlag.data[2] = self.contactFlag.data[0]
    self.contactFlag.data[3] = self.contactFlag.data[1]

  def targetTorqueCallback(self, data):
    self.targetTorque = np.array(list(data.data))

  def targetPosCallback(self, data):
    self.targetPos = np.array(list(data.data))

  def targetVelCallback(self, data):
    self.targetVel = np.array(list(data.data)) 

  def targetKpCallback(self, data):
    self.targetKp = np.array(list(data.data))

  def targetKdCallback(self, data):
    self.targetKd = np.array(list(data.data))

  def reset(self):
    # Set camera configuration
    self.cam.azimuth = 89.608063
    self.cam.elevation = -11.588379
    self.cam.distance = 5.0
    self.cam.lookat = np.array([0.0, 0.0, 1.5])

  def thread_spin(self):
    rclpy.spin(self.node)

  # def controller(self, model, data):
  #   self.data.ctrl[0] = 100
  #   pass


  def simulate(self):
    th_spin = Thread(target=HumanoidSim.thread_spin,args=(self,))
    th_spin.start()
    publish_time = self.data.time
    torque_publish_time = self.data.time
    sim_epoch_start = time.time()
    while not glfw.window_should_close(self.window):
      simstart = self.data.time

      while (self.data.time - simstart <= 1.0/60.0 and not self.pause_flag):
        if (self.pause_flag==False and self.pause_flag_last ==True):
          simState = Bool()
          simState.data = self.pause_flag
          self.pause_flag_last = self.pause_flag
          self.pubSimState.publish(simState)  
        if (time.time() - sim_epoch_start >= 1.0 / self.sim_rate):
          # MIT control
          self.data.ctrl[:] = self.targetTorque + self.targetKp * (self.targetPos - self.data.qpos[-12:]) + self.targetKd * (self.targetVel - self.data.qvel[-12:])
          # Step simulation environment
          mj.mj_step(self.model, self.data)
          # 按固定周期累加（扣除 step 本身的耗时），落后过多（如程序断点暂停）时重新对齐墙钟
          sim_epoch_start += 1.0 / self.sim_rate
          if time.time() - sim_epoch_start > 0.1:
            sim_epoch_start = time.time()

        
        if (self.data.time - publish_time >= 1.0 / 500.0):
          # * Publish joint positions and velocities
          jointsPosVel = Float32MultiArray()
          # get last 12 element of qpos and qvel
          qp = self.data.qpos[-12:].copy()
          qv = self.data.qvel[-12:].copy()
          jointsPosVel_ = np.concatenate((qp,qv))
          jointsPosVel.data = array.array( 'f' , jointsPosVel_.tolist())
          
          self.pubJoints.publish(jointsPosVel)
          # * Publish body pose
          bodyOdom = Odometry()
          pos = self.data.sensor('BodyPos').data.copy()

          #add imu bias
          # MuJoCo 传感器输出 (w,x,y,z)，scipy 需要 (x,y,z,w)，必须显式转换
          ori_wxyz = self.data.sensor('BodyQuat').data.copy()
          ori = R.from_quat(np.array([ori_wxyz[1], ori_wxyz[2], ori_wxyz[3], ori_wxyz[0]])).as_euler('xyz')
          ori += imu_eular_bias
          ori = R.from_euler('xyz', ori).as_quat()  # 返回 (x,y,z,w)

          vel = self.data.qvel[:3].copy()
          angVel = self.data.sensor('BodyGyro').data.copy()

          bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
          bodyOdom.pose.pose.position.x = pos[0]
          bodyOdom.pose.pose.position.y = pos[1]
          bodyOdom.pose.pose.position.z = pos[2]
          bodyOdom.pose.pose.orientation.x = ori[0]
          bodyOdom.pose.pose.orientation.y = ori[1]
          bodyOdom.pose.pose.orientation.z = ori[2]
          bodyOdom.pose.pose.orientation.w = ori[3]
          bodyOdom.twist.twist.linear.x = vel[0]
          bodyOdom.twist.twist.linear.y = vel[1]
          bodyOdom.twist.twist.linear.z = vel[2]
          bodyOdom.twist.twist.angular.x = angVel[0]
          bodyOdom.twist.twist.angular.y = angVel[1]
          bodyOdom.twist.twist.angular.z = angVel[2]
          self.pubOdom.publish(bodyOdom)

          bodyImu = Imu()
          acc = self.data.sensor('BodyAcc').data.copy()
          bodyImu.header.stamp = self.node.get_clock().now().to_msg()
          bodyImu.angular_velocity.x = angVel[0]
          bodyImu.angular_velocity.y = angVel[1]
          bodyImu.angular_velocity.z = angVel[2]
          bodyImu.linear_acceleration.x = acc[0]
          bodyImu.linear_acceleration.y = acc[1]
          bodyImu.linear_acceleration.z = acc[2]
          bodyImu.orientation.x = ori[0]
          bodyImu.orientation.y = ori[1]
          bodyImu.orientation.z = ori[2]
          bodyImu.orientation.w = ori[3]
          bodyImu.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          bodyImu.angular_velocity_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          bodyImu.linear_acceleration_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
          self.pubImu.publish(bodyImu)

          self.updateContactFlag()
          self.pubContactFlag.publish(self.contactFlag)

          publish_time = self.data.time

      if (self.data.time - torque_publish_time >= 1.0 / 40.0):
        
        targetTorque = Float32MultiArray()
        targetTorque.data = array.array( 'f' , self.data.ctrl[:].tolist())
        self.pubRealTorque.publish(targetTorque)
        torque_publish_time = self.data.time
        

      if self.data.time >= self.simend:
          break
      if self.pause_flag:
        # publish the state even if the simulation is paused
        # * Publish joint positions and velocities
        jointsPosVel = Float32MultiArray()
        
        # get last 12 element of qpos and qvel
        qp = self.data.qpos[-12:].copy()
        qv = self.data.qvel[-12:].copy()
        jointsPosVel_ = np.concatenate((qp,qv))
        jointsPosVel.data = array.array( 'f' , jointsPosVel_.tolist())
        

        self.pubJoints.publish(jointsPosVel)
        # * Publish body pose
        bodyOdom = Odometry()
        pos = self.data.sensor('BodyPos').data.copy()

        #add imu bias
        ori = self.data.sensor('BodyQuat').data.copy()
        ori = R.from_quat(ori).as_euler('xyz')
        ori += imu_eular_bias
        ori = R.from_euler('xyz', ori).as_quat()

        vel = self.data.qvel[:3].copy()
        angVel = self.data.sensor('BodyGyro').data.copy()
        bodyOdom.header.stamp = self.node.get_clock().now().to_msg()
        bodyOdom.pose.pose.position.x = pos[0]
        bodyOdom.pose.pose.position.y = pos[1]
        bodyOdom.pose.pose.position.z = pos[2]
        bodyOdom.pose.pose.orientation.x = ori[1]
        bodyOdom.pose.pose.orientation.y = ori[2]
        bodyOdom.pose.pose.orientation.z = ori[3]
        bodyOdom.pose.pose.orientation.w = ori[0]
        bodyOdom.twist.twist.linear.x = 0.0
        bodyOdom.twist.twist.linear.y = 0.0
        bodyOdom.twist.twist.linear.z = 0.0
        bodyOdom.twist.twist.angular.x = 0.0
        bodyOdom.twist.twist.angular.y = 0.0
        bodyOdom.twist.twist.angular.z = 0.0
        self.pubOdom.publish(bodyOdom)

        bodyImu = Imu()
        bodyImu.header.stamp = self.node.get_clock().now().to_msg()
        bodyImu.angular_velocity.x = 0.0
        bodyImu.angular_velocity.y = 0.0
        bodyImu.angular_velocity.z = 0.0
        bodyImu.linear_acceleration.x = 0.0
        bodyImu.linear_acceleration.y = 0.0
        bodyImu.linear_acceleration.z = 9.81
        bodyImu.orientation.x = ori[1]
        bodyImu.orientation.y = ori[2]
        bodyImu.orientation.z = ori[3]
        bodyImu.orientation.w = ori[0]
        self.pubImu.publish(bodyImu)
        
      # get framebuffer viewport
      
      viewport_width, viewport_height = glfw.get_framebuffer_size(
          self.window)
      viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)
      
      # Update scene and render
      mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                          mj.mjtCatBit.mjCAT_ALL.value, self.scene)
      mj.mjr_render(viewport, self.scene, self.context)
      
      # swap OpenGL buffers (blocking call due to v-sync)
      glfw.swap_buffers(self.window)

      # process pending GUI events, call GLFW callbacks
      glfw.poll_events()
      
    glfw.terminate()
    # 先 shutdown 让 rclpy.spin() 返回，再 join；否则 spin 线程永不退出，进程只能被 Ctrl+C 杀死
    rclpy.shutdown()
    th_spin.join()

def main():
    # ros init
    rclpy.init()
    hector_desc_path = get_prefix_path('humanoid_legged_description')
    xml_path = hector_desc_path + "/share/humanoid_legged_description/mjcf/humanoid_legged_control_.xml"


    sim = HumanoidSim(xml_path)#创建仿真节点
    sim.reset()
    
    sim.simulate()

    # rclpy 已在 simulate() 退出时 shutdown
    sim.node.destroy_node()

if __name__ == "__main__":
    main()