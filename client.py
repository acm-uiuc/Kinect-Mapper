import socket, sys

import pygame
from pygame.locals import *


pygame.init()
screen = pygame.display.set_mode((80, 80))
pygame.display.set_caption('Robbie wanders around SC')
pygame.mouse.set_visible(0)

joysticks = []
for i in range(0, pygame.joystick.get_count()):
    joysticks.append(pygame.joystick.Joystick(i))
    joysticks[-1].init()
    print "Detected joystick '",joysticks[-1].get_name(),"'"
clock = pygame.time.Clock()

HOST = sys.argv[1]
PORT = int(sys.argv[2])

cs = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
cs.connect((HOST, PORT))


speed = 8
done = False
while not done:
    for event in pygame.event.get():
        clock.tick(60)

        if event.type == JOYBUTTONUP:
          cs.send("stp\n")

        if event.type == KEYUP:
            if event.key in [K_KP8, K_KP2, K_KP4, K_KP6, K_s]:
                cs.send("stop\n")

        elif event.type == KEYDOWN:
          if event.key == K_ESCAPE:
            print("Exiting...")
            done = True
            cs.send("stop\n")
            sys.exit()

          if event.key == K_k: speed = min(128,  speed + 4)
          if event.key == K_j: speed = max(-128, speed - 4)
          if event.key == K_KP5: cs.send("stop\n")

        if (event.type == JOYBUTTONDOWN and event.button == 5):
          cs.send("stop\n")


        if ((event.type == JOYBUTTONDOWN and event.button == 3) or (event.type == KEYDOWN and event.key == K_UP)):
          cs.send("ahead " + repr(speed) + "\n")

        if ((event.type == JOYBUTTONDOWN and event.button == 0) or (event.type == KEYDOWN and event.key == K_DOWN)):
          cs.send("ahead " + repr(-speed)+ "\n")

        if ((event.type == JOYBUTTONDOWN and event.button == 1) or (event.type == KEYDOWN and event.key == K_RIGHT)):
          cs.send("turn " + repr(speed)  + "\n")

        if ((event.type == JOYBUTTONDOWN  and event.button == 2) or (event.type == KEYDOWN and event.key == K_LEFT)):
          cs.send("turn " + repr(-speed) + "\n")
