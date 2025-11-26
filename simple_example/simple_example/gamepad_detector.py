import pygame

pygame.init()
print("Joysticks:", pygame.joystick.get_count())
for i in range(pygame.joystick.get_count()):
    j = pygame.joystick.Joystick(i)
    j.init()
    print("Joystick:", j.get_name())
