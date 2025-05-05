#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

int main() {
  wb_robot_init();

  // Inicjalizacja silników
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Inicjalizacja sensorów odległości
  WbDeviceTag prox_sensors[8];
  char sensor_names[8][5] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  
  for (int i = 0; i < 8; i++) {
    prox_sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(prox_sensors[i], TIME_STEP);
  }

  // Pętla główna
  while (wb_robot_step(TIME_STEP) != -1) {
    double sensor_values[8];

    // Odczyt sensorów
    for (int i = 0; i < 8; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
      printf("Sensor %d: %f\n", i, sensor_values[i]);
    }

    // Detekcja ścian
    bool left_wall = sensor_values[5] > 80;
    bool front_wall = sensor_values[7] > 80 || sensor_values[0] > 80;

    double left_speed = 0.0;
    double right_speed = 0.0;

    // Logika nawigacji
    if (front_wall) {
      printf("Ściana przed nami — skręcamy w prawo\n");
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else if (left_wall) {
      printf("Ściana po lewej — jedziemy prosto\n");
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED;
    } else {
      printf("Brak ściany po lewej — skręcamy w lewo\n");
      left_speed = MAX_SPEED / 8;
      right_speed = MAX_SPEED;
    }

    // Ustawienie prędkości silników
    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
