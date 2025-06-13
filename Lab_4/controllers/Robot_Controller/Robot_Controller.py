from controller import Robot

# Parametry
TIMESTEP = 64
MAX_SPEED = 6.28
BALL_COLOR_THRESHOLD = 20
WHITE_LINE_THRESHOLD = 200
SEARCH_TIMEOUT = 50.0  

# Inicjalizacja robota
robot = Robot()

left_motor = robot.getMotor("LEFT_MOTOR")
right_motor = robot.getMotor("RIGHT_MOTOR")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

camera_front = robot.getDevice("camera_front")
camera_bottom = robot.getDevice("camera_bottom")
camera_front.enable(TIMESTEP)
camera_bottom.enable(TIMESTEP)

sensor_front = robot.getDevice("sensor_1")
sensor_front.enable(TIMESTEP)

width = camera_front.getWidth()
height = camera_front.getHeight()

search_time = 0.0
state = "SEARCH"  # inne: "PUSH", "REVERSE", "IDLE_AFTER_REVERSE", "DONE"
timer = 0.0
reverse_start = 0.0
idle_start = 0.0
# Funkcje pomocnicze
def detect_blue_ball(image):
    for x in range(0, width, 4):
        r = camera_front.imageGetRed(image, width, x, 22)
        g = camera_front.imageGetGreen(image, width, x, 22)
        b = camera_front.imageGetBlue(image, width, x, 22)
        if r > BALL_COLOR_THRESHOLD and g < BALL_COLOR_THRESHOLD and b < BALL_COLOR_THRESHOLD:
            return True, x
    return False, -1

def is_on_white_line(image):
    count = 0
    for y in range(0, height, 4):
        for x in range(0, width, 4):
            r = camera_bottom.imageGetRed(image, width, x, y)
            g = camera_bottom.imageGetGreen(image, width, x, y)
            b = camera_bottom.imageGetBlue(image, width, x, y)
            if r > WHITE_LINE_THRESHOLD and g > WHITE_LINE_THRESHOLD and b > WHITE_LINE_THRESHOLD:
                count += 1
    return count > 20

# Główna pętla
while robot.step(TIMESTEP) != -1:
    print(f"We are in state {state}")
    timer += TIMESTEP / 1000.0
    image_front = camera_front.getImage()
    image_bottom = camera_bottom.getImage()
    if state == "SEARCH":
        if is_on_white_line(image_bottom):
            state = "REVERSE"
            reverse_start = timer
        found, x = detect_blue_ball(image_front)
        print(f"Ball found: {found}, {x}, {width//3}")
        if found:
            print(sensor_front.getValue())
            search_time = 0
            if sensor_front.getValue()<500:
                state = "PUSH"
            if x > width // 3:
                left_speed = 0.25 * MAX_SPEED
                right_speed = 0.5 * MAX_SPEED
            elif x < (2 * width // 3):
                left_speed = 0.5 * MAX_SPEED
                right_speed = 0.25 * MAX_SPEED
            else:
                left_speed = right_speed = 0.5 * MAX_SPEED
                state = "PUSH"
        else:
            search_time += TIMESTEP / 1000.0
            left_speed = 0.3 * MAX_SPEED
            right_speed = -0.3 * MAX_SPEED  # obrót

        if search_time > SEARCH_TIMEOUT:
            state = "DONE"

    elif state == "PUSH":
        
        found, x = detect_blue_ball(image_front)
        if is_on_white_line(image_bottom):
            state = "REVERSE"
            reverse_start = timer
        elif x > width // 3 and not(sensor_front.getValue()<500):
            left_speed = 0.2 * MAX_SPEED
            right_speed = 0.5 * MAX_SPEED
        elif x < 2 * width // 3 and not(sensor_front.getValue()<500):
            left_speed = 0.5 * MAX_SPEED
            right_speed = 0.2 * MAX_SPEED
        else:
            left_speed = right_speed = 0.5 * MAX_SPEED

    elif state == "REVERSE":
        if timer - reverse_start > 2.5:
            state = "SEARCH"
        left_speed = -0.25 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED

    elif state == "DONE":
        left_speed = right_speed = 0.0

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
