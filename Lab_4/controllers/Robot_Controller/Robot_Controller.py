
from controller import Robot

if __name__ == "__main__":
    # create the Robot instance.
    robot = Robot()
    
    # get the time step of the current world.
    timestep = 64
    max_speed = 6.28
        
    
    # motor instances
    left_motor = robot.getMotor("LEFT_MOTOR")
    right_motor = robot.getMotor("RIGHT_MOTOR")
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0);
    
    # camera instances
    #Front
    camera_front = robot.getDevice("camera_front")
    camera_front.enable(timestep)
    camera_bottom = robot.getDevice("camera_bottom")
    camera_bottom.enable(timestep)
    width = camera_front.getWidth()
    height = camera_front.getHeight()
    
    # sensor instances
    #front it will be 3 sensors 
    sensors_front = robot.getDevice("sensor_1")
    sensors_front.enable(timestep)
    

    

    while robot.step(timestep) != -1:
    
        values = sensors_front.getValue()
        #print(values)
        left_speed = 0.5 * max_speed
        right_speed = 0.5 * max_speed
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        # image
        image_front = camera_front.getImage()
        image_bottom = camera_bottom.getImage()
        for y in range(height):
            for x in range(width):
                r = camera_front.imageGetRed(image_bottom, width, x, y)
                g = camera_front.imageGetGreen(image_bottom, width, x, y)
                b = camera_front.imageGetBlue(image_bottom, width, x, y)
                #print(r,g,b)
                
        
        
        pass
    

    