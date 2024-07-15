import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

import vlc
from ament_index_python.packages import get_package_share_directory
import os
from time import sleep
import threading


class PlaySound(Node):
    def __init__(self):
        super().__init__("play_sound_node")

        self.file1 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzer_Song.mp3")
        self.file2 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzer_V2_Song.mp3")
        self.file3 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzermusik_Song.mp3")

        self.track_1 = vlc.MediaPlayer(self.file1)
        self.track_2 = vlc.MediaPlayer(self.file2)
        self.track_3 = vlc.MediaPlayer(self.file3)

        self.track_1_duration = 31.8
        self.track_2_duration = 60.8
        self.track_3_duration = 60.8
        

        self.sub_ = self.create_subscription(Int32, "music", self.msgCallback, 10)

        
    def play_track(self, track, duration):
        try:

            track.play()
            sleep(duration)
            track.stop()
        except Exception as e:
            self.get_logger().error(f"Error play Track: {e}")

    def msgCallback(self, msg):

        play = msg.data        
        try:
            if play == 1:
                threading.Thread(target=self.play_track, args=(self.track_1, self.track_1_duration)).start()

            elif play == 2:
                threading.Thread(target=self.play_track, args=(self.track_2, self.track_2_duration)).start()

            elif play == 3:
                threading.Thread(target=self.play_track, args=(self.track_3, self.track_3_duration)).start()
            self.get_logger().info("Spiele Sound : %d" % msg.data)
        except Exception as e:
            self.get_logger().error(f"Error in msgCallback: {e}")

        self.get_logger().info("Spiele Sound : %d" % msg.data)



def main():
    rclpy.init()
    simple_subscriber = PlaySound()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()