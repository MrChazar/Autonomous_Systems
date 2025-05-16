#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <string.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28

typedef struct {
  double x;  // Pozycja x robota
  double z;  // Pozycja z robota
} Message;

int main() {
  wb_robot_init();

  // Inicjalizacja kamery
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  
  // Pobierz parametry obrazu
  int width = wb_camera_get_width(camera);
  int height = wb_camera_get_height(camera);
  bool notMove = 0;
  
  // inicjalizacja nazw
  const char *robot_name = wb_robot_get_name();
  int channel_send, channel_receive;
  if (strcmp(robot_name, "Maze-Crawler_1") == 0) {
    channel_send = 1;
    channel_receive = 2;
  } else if (strcmp(robot_name, "Maze-Crawler_2") == 0) {
    channel_send = 2;
    channel_receive = 1;
  } else {
    printf("Błąd: Nieznana nazwa robota %s\n", robot_name);
    return 1;
  }
  
  // Inicjalizacja urządzeń
  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

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
    double pos_x = 0.0, pos_z = 0.0;
    
    Message msg = {pos_x, pos_z};
    wb_emitter_send(emitter, &msg, sizeof(Message));
    printf("%s wysłał pozycję: [%.3f, %.3f]\n", robot_name, msg.x, msg.z);
    
    while (wb_receiver_get_queue_length(receiver) > 0) {
      const Message *received_msg = (const Message *)wb_receiver_get_data(receiver);
      printf("%s otrzymał pozycję: [%.3f, %.3f]\n", robot_name, received_msg->x, received_msg->z);
      wb_receiver_next_packet(receiver);  // Przejdź do następnej wiadomości
    }
   

    // Odczyt sensorów
    for (int i = 0; i < 8; i++) {
      sensor_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
      //printf("Sensor %d: %f\n", i, sensor_values[i]);
    }

    // Wczytanie obrazu
    const unsigned char *image = wb_camera_get_image(camera);

    int red_pixels = 0;
    
    // Będziemy sprawdzać piksel po pikselu xD
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        
        int r = wb_camera_image_get_red(image, width, x, y);
        int g = wb_camera_image_get_green(image, width, x, y);
        int b = wb_camera_image_get_blue(image, width, x, y);
        if (r > 200 && g < 100 && b < 100) {
          red_pixels++;
        }
      }
    }
    
    
    if (red_pixels > 1) { 
      printf("Wykryto czerwony kolor!");
      notMove = 1;
    }
  

    // Detekcja ścian
    bool left_wall = sensor_values[5] > 80;
    bool front_wall = sensor_values[7] > 80 || sensor_values[0] > 80;

    double left_speed = 0.0;
    double right_speed = 0.0;

    // Logika nawigacji
    if (front_wall && !notMove) {
      //printf("🔄 Ściana przed nami — skręcamy w prawo\n");
      left_speed = MAX_SPEED;
      right_speed = -MAX_SPEED;
    } else if (left_wall && !notMove) {
      //printf("⬆️ Ściana po lewej — jedziemy prosto\n");
      left_speed = MAX_SPEED;
      right_speed = MAX_SPEED;
    } else if(!notMove) {
      //printf("↪️ Brak ściany po lewej — skręcamy w lewo\n");
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
