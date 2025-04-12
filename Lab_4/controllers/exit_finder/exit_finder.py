"""exit_finder controller.
dokumentacja robocika: 
"""

from controller import Robot

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28

    # Silniki
    left_motor = robot.getMotor('left wheel motor')
    right_motor = robot.getMotor('right wheel motor')

    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))

    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    # Sensory odległości
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        sensor = robot.getDistanceSensor(sensor_name)
        sensor.enable(timestep)
        prox_sensors.append(sensor)

    # Pętla główna
    while robot.step(timestep) != -1:
        # Odczyty z sensorów
        sensor_values = [sensor.getValue() for sensor in prox_sensors]

        for ind, val in enumerate(sensor_values):
            print(f"Sensor {ind}: {val}")

        # Progi detekcji — można dostosować w zależności od sceny
        left_wall = sensor_values[5] > 80
        front_wall = sensor_values[7] > 80 or sensor_values[0] > 80  # lewy/przedni czujnik z przodu

        # Logika nawigacji
        if front_wall:
            print("🔄 Ściana przed nami — skręcamy w prawo")
            left_speed = max_speed
            right_speed = -max_speed
        elif left_wall:
            print("⬆️ Ściana po lewej — jedziemy prosto")
            left_speed = max_speed
            right_speed = max_speed
        else:
            print("↪️ Brak ściany po lewej — skręcamy w lewo")
            left_speed = max_speed / 8
            right_speed = max_speed

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    robot = Robot()
    run_robot(robot)
