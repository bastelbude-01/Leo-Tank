import vlc
from ament_index_python.packages import get_package_share_directory
import os
from time import sleep

file1 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzer_Song.mp3")
file2 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzer_V2_Song.mp3")
file3 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzermusik_Song.mp3")

p = vlc.MediaPlayer(file1)

p.play()
sleep(31.8)
p.stop()


# By Markus Messerschmidt
# pip3 install python-vlc
# Ausbau fähig!!