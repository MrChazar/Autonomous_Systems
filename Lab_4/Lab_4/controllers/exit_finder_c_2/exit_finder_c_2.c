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
#define COMM_TIMEOUT 100  // Limit cykli na komunikację (100 * 64ms = 6.4s)
#define GREEN_TIMEOUT_MS 5000  // 5 sekund na utratę zielonego elementu

int main() {
    wb_robot_init();

    // Pobierz nazwę robota
    const char *robot_name = wb_robot_get_name();
    int emit_channel, recv_channel;

    // Ustaw kanały na podstawie nazwy robota
    if (strcmp(robot_name, "Maze-Crawler_1") == 0) {
        emit_channel = 1;
        recv_channel = 2;
        printf("[%s] Inicjalizacja: Nadaje na kanale %d, odbiera na kanale %d\n", robot_name, emit_channel, recv_channel);
    } else {
        emit_channel = 2;
        recv_channel = 1;
        printf("[%s] Inicjalizacja: Nadaje na kanale %d, odbiera na kanale %d\n", robot_name, emit_channel, recv_channel);
    }

    // Inicjalizacja kamery
    WbDeviceTag camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, TIME_STEP);
    int width = wb_camera_get_width(camera);
    int height = wb_camera_get_height(camera);

    // Inicjalizacja emitera i odbiornika
    WbDeviceTag emitter = wb_robot_get_device("emitter");
    WbDeviceTag receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, TIME_STEP);
    wb_emitter_set_channel(emitter, emit_channel);
    wb_receiver_set_channel(receiver, recv_channel);

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
    int comm_counter = 0; 
    int no_green_timer = 0; 
    srand(time(NULL));

    while (wb_robot_step(TIME_STEP) != -1) {
        // Odczyt czujników odległości
        double sensor_values[8];
        for (int i = 0; i < 8; i++) {
            sensor_values[i] = wb_distance_sensor_get_value(prox_sensors[i]);
        }

        // Odczyt obrazu z kamery
        const unsigned char *image = wb_camera_get_image(camera);
        bool green_present = false;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int r = wb_camera_image_get_red(image, width, x, y);
                int g = wb_camera_image_get_green(image, width, x, y);
                int b = wb_camera_image_get_blue(image, width, x, y);
                if (r < 100 && g > 128 && b < 100) {
                    green_present = true;
                }
                if (r > 100 && r < 150 && g < 50 && b > 200) { // Fioletowy cel (0.5 0.0 1.0)
                    exit_found = true;
                }
            }
        }

        // Zarządzanie timeoutem zielonego elementu
        if (green_present) {
            no_green_timer = 0;
        } else {
            no_green_timer += TIME_STEP;
        }
        if (no_green_timer >= GREEN_TIMEOUT_MS && state != SEARCHING) {
            printf("[%s] Timeout zielonego elementu! Wracam do SEARCHING.\n", robot_name);
            state = SEARCHING;
            is_leader = false;
        }

        // Domyślne prędkości
        double left_speed = 0.0;
        double right_speed = 0.0;

        if (exit_found) {
            printf("[%s] Wykryto wyjście!\n", robot_name);
            state = SEARCHING;
            left_speed = 0.0;
            right_speed = 0.0;
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

            // Wykrycie innego robota
            if (green_present) {
                printf("[%s] Wykryto innego robota! Rozpoczynam komunikację.\n", robot_name);
                state = COMMUNICATING;
                comm_counter = 0;
                is_leader = (rand() % 2 == 0);
                const char *message = is_leader ? "LEADER" : "FOLLOWER";
                printf("[%s] Wysłano wiadomość: %s na kanale %d\n", robot_name, message, emit_channel);
                wb_emitter_send(emitter, message, strlen(message) + 1);
            }
        } else if (state == COMMUNICATING) {
            // Zatrzymaj robota podczas komunikacji
            left_speed = 0.0;
            right_speed = 0.0;

            // Sprawdź wiadomości
            while (wb_receiver_get_queue_length(receiver) > 0) {
                const char *message = (const char *)wb_receiver_get_data(receiver);
                printf("[%s] Odebrano wiadomość: %s na kanale %d\n", robot_name, message, recv_channel);
                if (strcmp(message, "LEADER") == 0 && !is_leader) {
                    state = FOLLOWER;
                    printf("[%s] Jestem podążającym!\n", robot_name);
                } else if (strcmp(message, "FOLLOWER") == 0 && is_leader) {
                    state = LEADER;
                    printf("[%s] Jestem liderem!\n", robot_name);
                } else {
                    printf("[%s] Ignoruję wiadomość: %s (niezgodna z rolą)\n", robot_name, message);
                }
                wb_receiver_next_packet(receiver);
            }

            // Ponowne wysyłanie wiadomości co 10 cykli
            if (comm_counter % 10 == 0) {
                const char *message = is_leader ? "LEADER" : "FOLLOWER";
                printf("[%s] Ponowne wysłanie wiadomości: %s na kanale %d\n", robot_name, message, emit_channel);
                wb_emitter_send(emitter, message, strlen(message) + 1);
            }

            // Timeout komunikacji
            comm_counter++;
            if (comm_counter > COMM_TIMEOUT) {
                printf("[%s] Timeout komunikacji! Wracam do SEARCHING.\n", robot_name);
                state = SEARCHING;
                is_leader = false;
            }
        } else if (state == LEADER) {
            // Algorytm "left-hand rule" dla lidera
            bool left_wall = sensor_values[5] > 80;
            bool front_wall = sensor_values[7] > 80 || sensor_values[0] > 80;

            if (front_wall && left_wall) {
                printf("[%s] Lider utknął! Zamiana ról.\n", robot_name);
                state = FOLLOWER;
                is_leader = false;
                const char *message = "FOLLOWER";
                printf("[%s] Wysłano wiadomość: %s na kanale %d\n", robot_name, message, emit_channel);
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

            // Wysłanie prędkości do podążającego
            double speeds[2] = {left_speed, right_speed};
            wb_emitter_send(emitter, speeds, sizeof(speeds));
            printf("[%s] Wysłano prędkości: left=%f, right=%f na kanale %d\n", robot_name, left_speed, right_speed, emit_channel);
        } else if (state == FOLLOWER) {
            // Podążanie za liderem
            if (green_present) {
                left_speed = MAX_SPEED * 0.75; // Wolniej niż lider
                right_speed = MAX_SPEED * 0.75;
            } else {
                left_speed = MAX_SPEED / 4;
                right_speed = -MAX_SPEED / 4; // Obrót w poszukiwaniu lidera
            }

            // Odbiór prędkości lub zamiany ról
            while (wb_receiver_get_queue_length(receiver) > 0) {
                const char *message = (const char *)wb_receiver_get_data(receiver);
                if (strlen(message) == sizeof(double) * 2) { // Sprawdzenie, czy to prędkości
                    const double *speeds = (const double *)message;
                    left_speed = speeds[0] * 0.75;
                    right_speed = speeds[1] * 0.75;
                    printf("[%s] Odebrano prędkości: left=%f, right=%f na kanale %d\n", robot_name, left_speed, right_speed, recv_channel);
                } else if (strcmp(message, "FOLLOWER") == 0) {
                    printf("[%s] Lider chce zamiany! Staję się liderem.\n", robot_name);
                    state = LEADER;
                    is_leader = true;
                    const char *new_message = "LEADER";
                    printf("[%s] Wysłano wiadomość: %s na kanale %d\n", robot_name, new_message, emit_channel);
                    wb_emitter_send(emitter, new_message, strlen(new_message) + 1);
                }
                wb_receiver_next_packet(receiver);
            }
        }

        // Ustawienie prędkości silników
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
    }

    wb_robot_cleanup();
    return 0;
}