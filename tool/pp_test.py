# Test (or demonstrate) the PP logic
# Move forwards and backwards with W/S, rotate with A,D
# Move your cursor to change the waypoint target

import pygame
import math
import numpy as np

def normalizeAngle(angle):
  while(angle > math.pi):
    angle -= (2 * math.pi)

  while(angle < -math.pi):
    angle += (2 * math.pi)

  return angle;

def calculateSteeringTarget(target, pos, car_yaw, lookforward, wheelbase):
  SteerTarget = normalizeAngle(math.atan2(target[1] - pos[1], target[0] - pos[0]) - car_yaw);
  wheelRotation = math.atan2(2 * wheelbase * math.sin(SteerTarget) / (lookforward), 1)
  return wheelRotation;

def curv_from_steer(wheel_ang_rad, L, FBAL):
  LR = L * FBAL;
  LF = L * (1 - FBAL);

  beta = math.atan(LR * math.tan(wheel_ang_rad) / L);
  k = (math.tan(wheel_ang_rad) * math.cos(beta) - math.sin(beta)) / LF
  return k

L = 1.5421
FBAL = 0.46

# pygame setup
pygame.init()
screen = pygame.display.set_mode((1280, 720))
clock = pygame.time.Clock()
running = True


car_pos = np.array([0, 0])
theta = 0
VEL = 1
THETA_VEL = 0.05

while running:
  for event in pygame.event.get():
    if event.type == pygame.QUIT:
      running = False
  
  kpress = pygame.key.get_pressed()
  ds = int(kpress[pygame.key.key_code('w')]) - int(kpress[pygame.key.key_code('s')])
  dtheta = int(kpress[pygame.key.key_code('d')]) - int(kpress[pygame.key.key_code('a')])

  theta = theta + dtheta * THETA_VEL
  car_pos = car_pos + np.array([math.cos(theta), math.sin(theta)]) * ds * VEL
  fwd = np.array([math.cos(theta), math.sin(theta)])
  left = np.array([-fwd[1], fwd[0]])

  tgt = np.array(pygame.mouse.get_pos())

  steer = calculateSteeringTarget(tgt, car_pos, theta, np.linalg.norm(car_pos - tgt), L)
  k = curv_from_steer(steer, L, FBAL)

  print(k)
  screen.fill("black")
  r = (1 / k)
  center = car_pos + left * r
  pygame.draw.circle(screen, 'white', center, abs(r), 3)

  pygame.draw.line(screen, 'white', car_pos, car_pos + fwd * 15, width = 7)
  pygame.draw.circle(screen, 'gray', tgt, 3)

  pygame.display.flip()
  clock.tick(30)  # limits FPS to 60

pygame.quit()
