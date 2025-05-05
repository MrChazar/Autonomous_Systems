#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

typedef enum { SEARCHING, LEADER, FOLLOWER } RobotMode;

int main() {
  wb_robot_init();
  srand(time(NULL));

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);

  WbDeviceTag prox_sensors[8];
  char sensor_names[8][5] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  for (int i = 0; i < 8; i++) {
    prox_sensors[i] = wb_robot_get_device(sensor_names[i]);
    wb_distance_sensor_enable(prox_sensors[i], TIME_STEP);
  }

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  int my_id = rand() % 10000;
  RobotMode mode = SEARCHING;
  bool communication_started = false;

  while (wb_robot_step(TIME_STEP) != -1) {
    double sensor_values[8];
    for (int i = 0; i < 8; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
    }

    bool left_wall = sensor_values[5] > 80;
    bool front_wall = sensor_values[7] > 80 || sensor_values[0] > 80;
    bool close_robot = sensor_values[6] > 100 || sensor_values[1] > 100;

    double left_speed = 0.0;
    double right_speed = 0.0;

    // Odbieranie komunikatów
    if (wb_receiver_get_queue_length(receiver) > 0) {
      const void *data = wb_receiver_get_data(receiver);
      int other_id;
      memcpy(&other_id, data, sizeof(int));

      if (!communication_started) {
        communication_started = true;
        if (my_id < other_id) {
          mode = LEADER;
          printf("Jestem liderem (ID %d < %d)\n", my_id, other_id);
        } else {
          mode = FOLLOWER;
          printf("Jestem podążającym (ID %d > %d)\n", my_id, other_id);
        }
      }

      wb_receiver_next_packet(receiver);
    }

    // Jeśli robot wykrył innego i nie rozpoczęto komunikacji — wyślij swoje ID
    if (close_robot && !communication_started) {
      printf("Wykryto innego robota — wysyłam swoje ID: %d\n", my_id);
      wb_emitter_send(emitter, &my_id, sizeof(int));
    }

    // Logika ruchu
    if (mode == SEARCHING || mode == LEADER) {
      if (front_wall) {
        left_speed = MAX_SPEED;
        right_speed = -MAX_SPEED;
      } else if (left_wall) {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      } else {
        left_speed = MAX_SPEED / 8;
        right_speed = MAX_SPEED;
      }
    } else if (mode == FOLLOWER) {
      if (close_robot) {
        left_speed = MAX_SPEED * 0.5;
        right_speed = MAX_SPEED * 0.5;
      } else {
        left_speed = MAX_SPEED;
        right_speed = MAX_SPEED;
      }
    }

    // Jeśli lider utknął — zamiana ról
    if (mode == LEADER && front_wall && left_wall) {
      printf("Lider utknął — zmieniamy role\n");
      mode = FOLLOWER;
    } else if (mode == FOLLOWER && !front_wall && !left_wall) {
      printf("Podążający przejmuje prowadzenie\n");
      mode = LEADER;
    }

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
