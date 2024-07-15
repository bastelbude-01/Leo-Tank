import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import vlc
from ament_index_python.packages import get_package_share_directory
import os
from time import sleep


class Subscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")

        self.file1 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzer_Song.mp3")
        self.file2 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzer_V2_Song.mp3")
        self.file3 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzermusik_Song.mp3")

        

        self.sub_ = self.create_subscription(Int32, "music", self.msgCallback, 10)

        

    def msgCallback(self, msg):

        play = msg.data

        self.track_1 = vlc.MediaPlayer(self.file1)
        self.track_2 = vlc.MediaPlayer(self.file2)
        self.track_3 = vlc.MediaPlayer(self.file3)

        if play == 1:
            self.track_1.play()
            sleep(31.8)
            self.track_1.stop()

        elif play == 2:
            self.track_2.play()
            sleep(31.8)
            self.track_2.stop()

        elif play == 3:
            self.track_3.play()
            sleep(31.8)
            self.track_3.stop()

        #self.get_logger().info("Spiele Sound : %s" % msg.data)



def main():
    rclpy.init()
    simple_subscriber = Subscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()