import pygame
from ament_index_python.packages import get_package_share_directory
import os
from time import sleep

file1 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzer_Song.mp3")
file2 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzer_V2_Song.mp3")
file3 = os.path.join(get_package_share_directory("tank_utils"),"files","Panzermusik_Song.mp3")


pygame.init()

pygame.mixer.music.load(file1) # file laden
pygame.mixer.music.play(-1,0.0) # 0 = 1x Abspielen/ -1 = Loop/ 3 = 4x abspielen // startzeitpunkt

pygame.mixer.music.set_volume(.6)




# By Markus Messerschmidt
# pip3 install pygame
# is nicht gut