#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define COMMUNICATION_CHANNEL 1

int main() {
    wb_robot_init();
    
    const char *robot_name = wb_robot_get_name();

    // Inicjalizacja kamery
    WbDeviceTag camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, TIME_STEP);
    int width = wb_camera_get_width(camera);
    int height = wb_camera_get_height(camera);

    // Inicjalizacja emitera i odbiornika
    WbDeviceTag emitter = wb_robot_get_device("emitter");
    WbDeviceTag receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, TIME_STEP);
    
    if(strcmp(robot_name, "Maze-Crawler_1") == 0)
    {
      wb_emitter_set_channel(emitter, 1);
      wb_receiver_set_channel(receiver, 2);
    }
    else{
      wb_emitter_set_channel(emitter, 2);
      wb_receiver_set_channel(receiver, 1);
    }
    
    // Inicjalizacja silników
    WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
    WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    wb_motor_set_velocity(left_motor, 0.0);
    wb_motor_set_velocity(right_motor, 0.0);

    // Inicjalizacja czujników odległości
    WbDeviceTag prox_sensors[8];
    char sensor_names[8][5] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
    for (int i = 0; i < 8; i++) {
        prox_sensors[i] = wb_robot_get_device(sensor_names[i]);
        wb_distance_sensor_enable(prox_sensors[i], TIME_STEP);
    }

    // Stany robota
    enum State { SEARCHING, COMMUNICATING, LEADER, FOLLOWER };
    enum State state = SEARCHING;
    bool is_leader = false;
    bool exit_found = false;
    srand(time(NULL)); 

    while (wb_robot_step(TIME_STEP) != -1) {
        
        // Odczyt czujników odległości
        double sensor_values[8];
        for (int i = 0; i < 8; i++) {
            sensor_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
        }

        // Odczyt obrazu z kamery
        const unsigned char *image = wb_camera_get_image(camera);
        int green_pixels = 0;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int r = wb_camera_image_get_red(image, width, x, y);
                int g = wb_camera_image_get_green(image, width, x, y);
                int b = wb_camera_image_get_blue(image, width, x, y);
                if (r < 100 && g > 128 && b < 100) {
                    green_pixels++;
                }
               if (r >= 128 && g <= 0 && b >= 128 ) {
                    exit_found = true;
                }
            }
        }

       
        double left_speed = 0.0;
        double right_speed = 0.0;
        printf("%d", state);
        if (exit_found) {
            printf("Wykryto wyjście!\n");
            state = SEARCHING;
        } else if (state == SEARCHING) {
            bool left_wall = sensor_values[5] > 80;
            bool front_wall = sensor_values[7] > 80 || sensor_values[0] > 80;

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

            if (green_pixels > 1) {
                printf("Wykryto innego robota! Rozpoczynam komunikację.\n");
                state = COMMUNICATING;
                is_leader = (rand() % 2 == 0);
                const char *message = is_leader ? "LEADER" : "FOLLOWER";
                wb_emitter_send(emitter, message, strlen(message) + 1);
            }
        } else if (state == COMMUNICATING) {
            if (wb_receiver_get_queue_length(receiver) > 0) {
                const char *message = (const char *)wb_receiver_get_data(receiver);
                if (strcmp(message, "LEADER") == 0 && !is_leader) {
                    state = FOLLOWER;
                    printf("Jestem podążającym!\n");
                } else if (strcmp(message, "FOLLOWER") == 0 && is_leader) {
                    state = LEADER;
                    printf("Jestem liderem!\n");
                }
                wb_receiver_next_packet(receiver);
            }
        } else if (state == LEADER) {
            bool left_wall = sensor_values[5] > 80;
            bool front_wall = sensor_values[7] > 80 || sensor_values[0] > 80;

            if (front_wall && left_wall) {
                printf("Lider utknął! Zamiana ról.\n");
                state = FOLLOWER;
                is_leader = false;
                const char *message = "FOLLOWER";
                wb_emitter_send(emitter, message, strlen(message) + 1);
            } else if (front_wall) {
                left_speed = MAX_SPEED;
                right_speed = -MAX_SPEED;
            } else if (left_wall) {
                left_speed = MAX_SPEED;
                right_speed = MAX_SPEED;
            } else {
                left_speed = MAX_SPEED / 8;
                right_speed = MAX_SPEED;
            }
        } else if (state == FOLLOWER) {
            if (green_pixels > 1) {
                left_speed = MAX_SPEED * 0.5;
                right_speed = MAX_SPEED * 0.5;
            } else {
                left_speed = MAX_SPEED / 4;
                right_speed = -MAX_SPEED / 4;
            }

         
            if (wb_receiver_get_queue_length(receiver) > 0) {
                const char *message = (const char *)wb_receiver_get_data(receiver);
                if (strcmp(message, "FOLLOWER") == 0) {
                    printf("Lider chce zamiany! Staję się liderem.\n");
                    state = LEADER;
                    is_leader = true;
                    const char *new_message = "LEADER";
                    wb_emitter_send(emitter, new_message, strlen(new_message) + 1);
                }
                wb_receiver_next_packet(receiver);
            }
        }

   
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
    }

    wb_robot_cleanup();
    return 0;
}