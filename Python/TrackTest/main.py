
import math
import sys
import neat
import pygame as py



WIDTH, HEIGHT = (1920, 1080)

CAR_SIZE_X, CAR_SIZE_Y = (60, 60)

BORDER_COLOR = (255, 255, 255, 255)

MAX_SPEED = 20
STREERING_ANGLE = 2


current_generation = 0
class Car:
    def __init__(self):
        self.sprite = py.image.load('./car.png').convert()
        self.sprite = py.transform.scale(self.sprite, (CAR_SIZE_X, CAR_SIZE_Y))
        self.rotated_sprite = self.sprite

        self.position = [830, 920]
        self.angle = 0
        self.speed = 0

        self.center = [self.position[0] + CAR_SIZE_X / 2, self.position[1] + CAR_SIZE_Y / 2]

        self.radars = []
        self.drawing_radars = []

        self.alive = True
        
        self.distance = 0 #afstand afgelegd
        self.time = 0 #tijd afgelegd

        self.speed_set = False

    def draw(self, screen):
        screen.blit(self.rotated_sprite, self.position)
        self.draw_radar(screen)

    def draw_radar(self, screen):
            for radar in self.radars:
                position = radar[0]
                py.draw.line(screen, (0 ,255, 0), self.center, position)
                py.draw.circle(screen, (0 ,255, 0), position, 5)

    def check_collision(self, game_map):
        self.alive = True
        for point in self.corners:
            if game_map.get_at((int(point[0]), int(point[1]))) == BORDER_COLOR:
                self.alive = False
                break 

    def radar(self, degree, game_map):
        length = 0
        x = int(self.center[0] + math.cos(math.radians(360 - (self.angle + degree))) * length)                   
        y = int(self.center[1] + math.sin(math.radians(360 - (self.angle + degree))) * length)   

        while not game_map.get_at((x, y)) == BORDER_COLOR and length < 300:
            length = length + 1
            x = int(self.center[0] + math.cos(math.radians(360 - (self.angle + degree))) * length)
            y = int(self.center[1] + math.sin(math.radians(360 - (self.angle + degree))) * length)  

        dist = int(math.sqrt(math.pow(x - self.center[0], 2) + math.pow(y - self.center[1], 2)))     
        self.radars.append([(x, y), dist])

    def update(self, game_map):
        self.rotated_sprite = self.rotate_center(self.sprite, self.angle)
        self.position[0] += math.cos(math.radians(360 - self.angle)) * self.speed
        self.position[0] = max(self.position[0], 10)
        self.position[0] = min(self.position[0], WIDTH - 100)
        self.position[1] += math.sin(math.radians(360 - self.angle)) * self.speed
        self.position[1] = max(self.position[1], 10)
        self.position[1] = min(self.position[1], HEIGHT - 100)

        if not self.speed_set:
            self.speed = 20
            self.speed_set = True

        self.center = [int(self.position[0] + CAR_SIZE_X / 2), int(self.position[1] + CAR_SIZE_Y / 2)]
        self.distance += self.speed
        self.time += 1

        length = 0.5 * CAR_SIZE_X
        left_top = [self.center[0] + math.cos(math.radians(360 - (self.angle + 30))) * length, self.center[1] + math.sin(math.radians(360 - (self.angle + 30))) * length]
        right_top = [self.center[0] + math.cos(math.radians(360 - (self.angle + 150))) * length, self.center[1] + math.sin(math.radians(360 - (self.angle + 150))) * length]
        left_bottom = [self.center[0] + math.cos(math.radians(360 - (self.angle + 210))) * length, self.center[1] + math.sin(math.radians(360 - (self.angle + 210))) * length]
        right_bottom = [self.center[0] + math.cos(math.radians(360 - (self.angle + 330))) * length, self.center[1] + math.sin(math.radians(360 - (self.angle + 330))) * length]
        self.corners = [left_top, right_top, left_bottom, right_bottom]

        self.check_collision(game_map)
        self.radars.clear()

        for angle in [-90, 120, 45]:
            self.radar(angle, game_map)

    def get_radar_dist(self):
        distances = [0, 0, 0, 0 ,0]
        radars = self.radars
        for index, radar in enumerate(radars):
            distances[index] = int(radar[1] / 30)
        return distances    

    def check_isAlive(self):
        return self.alive

    def move_forward(self):    
        self.speed = min(self.speed + self.acceleration, self.max_speed)
    def move_reverse(self):
        self.speed = max(self.speed - self.acceleration, -self.max_speed)    

    def reduce_speed(self):
        self.speed = max(self.speed - self.acceleration / 2, 0)

    def rotate_center(self, image, angle):
        rectangle = image.get_rect()
        rotated_image = py.transform.rotate(image, angle)
        rotated_rectangle = rectangle.copy()
        rotated_rectangle.center = rotated_image.get_rect().center
        rotated_image = rotated_image.subsurface(rotated_rectangle).copy()
        return rotated_image       

    def get_reward(self):
        return self.distance / (CAR_SIZE_X / 2)

# def user_game():
#     run = True
    
#     py.init()
#     screen = py.display.set_mode((WIDTH, HEIGHT))
#     clock = py.time.Clock()
#     game_map = py.image.load('assets/map3.png').convert()
#     car = Car()
#     while run:
#         for event in py.event.get():
#             if event.type == py.QUIT:
#                 sys.exit(0)
#         keys = py.key.get_pressed()
#         moved = False
#         if keys[py.K_UP]:
#             moved = True
#             car.move_forward()
#         if keys[py.K_DOWN]:
#             moved = True
#             car.move_reverse()    
#         if keys[py.K_LEFT]:
#             car.angle += rotation_vel
#         if keys[py.K_RIGHT]:
#             car.angle -= rotation_vel
#         if not moved:
#             car.reduce_speed()    
#         car.update(game_map)      
#         screen.blit(game_map, (0, 0))
#         car.draw(screen)
#         py.display.flip() 
#         clock.tick(60)   
#         if not car.alive:
#             break
    
            
def eval_genomes(genomes, config):
    nets = []
    cars = []

    py.init()
    screen = py.display.set_mode((WIDTH, HEIGHT))

    for i, g in genomes:
        net = neat.nn.FeedForwardNetwork.create(g, config)
        nets.append(net)
        g.fitness = 0
        
        cars.append(Car())

    clock = py.time.Clock()
    generation_font = py.font.SysFont("Arial", 30) 
    alive_font = py.font.SysFont("Arial", 20)
    game_map = py.image.load('./map3.png').convert()

    global current_generation 
    current_generation += 1
    counter = 0

    while True:
        for event in py.event.get():
            if event.type == py.QUIT:
                sys.exit(0)

        for i, car in enumerate(cars):
            output = nets[i].activate(car.get_radar_dist())
            choice = output.index(max(output))
            if choice == 0:
                car.angle += 10 
            elif choice == 1:
                car.angle -= 10 
            elif choice == 2:
                if(car.speed - 2 >= 12):
                    car.speed -= 2 
            else:
                car.speed = max(car.speed + 2, 10)

        still_alive_cars = 0
        for i, car in enumerate(cars):
            if car.check_isAlive():
                still_alive_cars += 1
                car.update(game_map)
                genomes[i][1].fitness += car.get_reward()
                if genomes[i][1].fitness == 0:
                    car.alive = False

        if still_alive_cars == 0:
            break

        counter += 1
        if counter == 30 * 40:
            break

        screen.blit(game_map, (0, 0))

        for car in cars:
            if car.check_isAlive():
                car.draw(screen)


        # show info
        text = generation_font.render("Generation: " + str(current_generation), True, (0,0,0))
        text_rect = text.get_rect()
        text_rect.center = (1800, 10)
        screen.blit(text, text_rect)

        text = alive_font.render("Still Alive: " + str(still_alive_cars), True, (0, 0, 0))
        text_rect = text.get_rect()
        text_rect.center = (1800, 30)
        screen.blit(text, text_rect)

        py.display.flip()
        clock.tick(60) # 60 FPS        



if __name__ == '__main__':
    config_path = './config.txt'
    config = neat.config.Config(neat.DefaultGenome,
                                neat.DefaultReproduction,
                                neat.DefaultSpeciesSet,
                                neat.DefaultStagnation,
                                config_path)

    population = neat.Population(config)

    population = neat.Population(config)
    population.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    population.add_reporter(stats)
    population.run(eval_genomes, 1000)
