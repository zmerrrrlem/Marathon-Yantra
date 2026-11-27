#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point
from urdf_parser_py.urdf import URDF
from visualization_msgs.msg import Marker

class CoMCalculator(Node):

  def __init__(self):
    super().__init__('com_calculation')
    self.base_link_frame = self.declare_parameter("base_link_frame", "base_link_frame").value
    self.tf_prefix = self.declare_parameter("tf_prefix", "").value
    self.Mass = 0

    # Get robot description from URDF
    robot = URDF.from_parameter_server()
    self.links = robot.link_map

    # Delete links that contain no mass description
    unnecessary_links = []
    for link in self.links:
      if self.links[link].inertial is None:
        unnecessary_links.append(link)

    for link in unnecessary_links:
      del self.links[link]

    # Calculate the total mass of the robot
    for link in self.links:
      self.Mass += self.links[link].inertial.mass

    self.get_logger().info(f"Mass of robot is {self.Mass}")
    self.calculator()

  def calculator(self):
    # Initializations for tf and marker
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, self)
    zu_transformieren = PointStamped()
    marker = Marker()
    marker.header.frame_id = self.base_link_frame
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.pose.orientation.w = 1.0
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.scale.x = 0.03
    marker.scale.y = 0.03
    marker.scale.z = 0.03
    pub = self.create_publisher(Marker, 'com', 1)

    rate = self.create_rate(1)
    self.get_clock().sleep_for(1)

    # Loop for calculating the CoM while the robot is not shutdown
    while rclpy.ok():
      x = 0
      y = 0
      z = 0
      for link in self.links:
        try:
          # Get transformation matrix of link
          trans = tf_buffer.lookup_transform(self.base_link_frame, self.tf_prefix + link, rclpy.time.Time())
          # Transform CoM of link
          zu_transformieren.point.x = self.links[link].inertial.origin.xyz[0]
          zu_transformieren.point.y = self.links[link].inertial.origin.xyz[1]
          zu_transformieren.point.z = self.links[link].inertial.origin.xyz[2]
          zu_transformieren.header.frame_id = self.tf_prefix + link
          zu_transformieren.header.stamp = self.get_clock().now().to_msg()
          transformiert = do_transform_point(zu_transformieren, trans)
          # Calculate part of CoM equation depending on link
          x += self.links[link].inertial.mass * transformiert.point.x
          y += self.links[link].inertial.mass * transformiert.point.y
          z += self.links[link].inertial.mass * transformiert.point.z
        except (LookupException, ConnectivityException, ExtrapolationException) as err:
          self.get_logger().error(f"TF error in COM computation: {err}")

      # Finish CoM calculation
      x = x / self.Mass
      y = y / self.Mass
      z = z / self.Mass

      # Send CoM position to RViZ
      marker.pose.position.x = x
      marker.pose.position.y = y
      marker.pose.position.z = z
      pub.publish(marker)

      try:
        rate.sleep()
      except rclpy.exceptions.ROSInterruptException:
        self.get_logger().warn("Interrupted during sleep. Shutting down.")

def main(args=None):
  rclpy.init(args=args)
  calc = CoMCalculator()
  rclpy.spin(calc)
  calc.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()