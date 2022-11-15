
import pickle
import neat

MAX_SPEED = 10
STEERING_ANGLE = 10
ACCELERATION = 2

class Robot_car():

    def __init__(self):
        self.speed = MAX_SPEED
        self.angle = 0
        self.acceleration = ACCELERATION
        self.max_speed = MAX_SPEED
        self.steering_angle = STEERING_ANGLE

    def move_forward(self):    
        self.speed = min(self.speed + self.acceleration, self.max_speed)

    def move_reverse(self):
        if(self.speed - 2 >= 12):
            self.speed -= 2   

    def rotate(self, angle, d):
        if d < 0:
            self.angle -= angle + self.steering_angle
        if d > 0:
            self.angle += angle + self.steering_angle    


def run_robot(inputs, network_file_path, neat_config):
    with open(network_file_path, 'rb') as f:
        genome = pickle.load(f)
    f.close()
    config = neat.config.Config(neat.DefaultGenome,
                                neat.DefaultReproduction,
                                neat.DefaultSpeciesSet,
                                neat.DefaultStagnation,
                                neat_config)

    network = neat.nn.FeedForwardNetwork.create(genome, config)
    output = network.activate(inputs)
    choice = output.index(max(output))

    car = Robot_car()

    if choice == 0:
         car.rotate(output[0], 1)
    elif choice == 1:
         car.rotate(output[1], -1)
    elif choice == 2:
         car.move_reverse()
    else:
        car.move_forward()

    outputs = [car.speed, car.angle]    
    return outputs