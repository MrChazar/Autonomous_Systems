#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define TIME_STEP 32
#define WALL_HEIGHT 0.25
#define WALL_THICKNESS 0.01
#define SCALE 0.01  // 1 SVG unit = 1 cm = 0.01 m

double x_min = 1e9, x_max = -1e9, y_min = 1e9, y_max = -1e9;

void update_bounds(double x1, double y1, double x2, double y2) {
  if (x1 < x_min) x_min = x1;
  if (x2 < x_min) x_min = x2;
  if (x1 > x_max) x_max = x1;
  if (x2 > x_max) x_max = x2;
  if (y1 < y_min) y_min = y1;
  if (y2 < y_min) y_min = y2;
  if (y1 > y_max) y_max = y1;
  if (y2 > y_max) y_max = y2;
}

void set_robot_position() {
  WbNodeRef robot = wb_supervisor_node_get_from_def("Maze-Crawler");
  if (!robot) {
    printf("Nie znaleziono robota E-puck\n");
    return;
  }
  WbFieldRef translation_field = wb_supervisor_node_get_field(robot, "translation");
  WbFieldRef rotation_field = wb_supervisor_node_get_field(robot, "rotation");

  double width = (x_max - x_min) * SCALE;

  double pos_x = (x_min * SCALE) + 0.425 * width;
  double pos_z = y_min * SCALE;
  double pos_y = 0.021;  // wysokość robota nad podłożem

  const double position[3] = {pos_x, pos_y, pos_z};
  const double rotation[4] = {0, 1, 0, 0};  // skierowany "w górę" labiryntu

  wb_supervisor_field_set_sf_vec3f(translation_field, position);
  wb_supervisor_field_set_sf_rotation(rotation_field, rotation);
}

void add_goal(WbFieldRef children_field) {
  double width = (x_max - x_min) * SCALE;

  double goal_x = ((x_max * SCALE) + 0.4 * width)/2.5;
  double goal_z = y_max * SCALE;
  double goal_y = 1.4;  // nad podłożem

  char goal_def[1024];
  snprintf(goal_def, sizeof(goal_def),
    "Transform { \
       translation %f %f %f \
       children [ \
         Shape { \
           appearance Appearance { \
             material Material { diffuseColor 1 0 0 } \
           } \
           geometry Sphere { radius 0.03 } \
         } \
       ] \
       name \"maze_goal\" \
     }",
    goal_x, goal_z, goal_y);

  wb_supervisor_field_import_mf_node_from_string(children_field, -1, goal_def);
}

void add_wall(double x1, double y1, double x2, double y2, WbFieldRef children_field) {
  update_bounds(x1, y1, x2, y2);

  double dx = (x2 - x1) * SCALE;
  double dy = (y2 - y1) * SCALE;
  double length = sqrt(dx * dx + dy * dy);
  double angle = atan2(dy, dx);

  double cx = ((x1 + x2) / 2.0) * SCALE;
  double cy = ((y1 + y2) / 2.0) * SCALE;
  double cz = WALL_HEIGHT / 2.0 + 1.37;

  char wall_def[1024];
  snprintf(wall_def, sizeof(wall_def),
    "Transform { \
       translation %f %f %f \
       rotation 0 0 1 %f \
       children [ \
         Shape { \
           appearance Appearance { \
             material Material { diffuseColor 0.2 0.2 0.8 } \
           } \
           geometry Box { size %f %f %f } \
         } \
       ] \
       boundingObject Box { size %f %f %f } \
       name \"maze_wall\" \
     }",
    cx, cy, cz, -angle,
    length+WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT,
    length+WALL_THICKNESS, WALL_THICKNESS, WALL_HEIGHT);
    
  wb_supervisor_field_import_mf_node_from_string(children_field, -1, wall_def);
}

void build_maze_from_svg(const char *filename, WbFieldRef children_field) {
  FILE *file = fopen(filename, "r");
  if (!file) {
    printf("Nie można otworzyć pliku: %s\n", filename);
    return;
  }

  char line[512];
  while (fgets(line, sizeof(line), file)) {
    if (strstr(line, "<line")) {
      double x1, y1, x2, y2;
      if (sscanf(line, " <line x1=\"%lf\" y1=\"%lf\" x2=\"%lf\" y2=\"%lf\" />", &x1, &y1, &x2, &y2) == 4) {
        add_wall(x1, y1, x2, y2, children_field);
      }
    }
  }

  fclose(file);
}

int main() {
  wb_robot_init();

  WbNodeRef root = wb_supervisor_node_get_root();
  WbFieldRef children = wb_supervisor_node_get_field(root, "children");

  double time = 0.0;
  while (wb_robot_step(TIME_STEP) != -1) {
    time += TIME_STEP / 1000.0;
    if (time > 2.0) {
      build_maze_from_svg("maze.svg", children);
      set_robot_position();
      add_goal(children);
      break;
    }
  }
  wb_robot_cleanup();
  return 0;
}
