#include <webots/robot.h>
#include <webots/supervisor.h>

void create_wall(double x, double y, double z, double rx, double ry, double rz, double angle) {
  WbNodeRef root = wb_supervisor_node_get_root();
  WbFieldRef children = wb_supervisor_node_get_field(root, "children");

  char wall_def[1024];
  printf(wall_def, sizeof(wall_def),
    "Solid { \
      name \"wall\" \
      translation %.5f %.5f %.5f \
      rotation %.5f %.5f %.5f %.5f \
      children [ \
        Shape { \
          appearance Appearance { \
            material Roughcast { \
              colorOverride 1 1 1 \
              TextureTransform { \
                center 0 0 \
                rotation 0 \
                scale 2.4 1 \
                translation 0 0 \
              } \
              IBLStrength 1 \
            } \
          } \
          geometry Box { size 0.01 0.25 0.1 } \
        } \
      ] \
    }",
    x, y, z, rx, ry, rz, angle
  );

  wb_supervisor_field_import_mf_node_from_string(children, -1, wall_def);
}

int main() {
  wb_robot_init();

  // Tworzymy kilka ścian — przykładowy labirynt
  create_wall(0.0, -0.08486, 1.0, 0, 0, -1, 0.785399);  // Ściana pod kątem 45°
  create_wall(1.0, -0.08486, 1.5, 0, 0, -1, 0.0);        // Ściana pozioma
  create_wall(2.0, -0.08486, 0.5, 0, 0, -1, 1.5708);     // Ściana pionowa
  create_wall(1.5, -0.08486, -0.5, 0, 0, -1, 0.785399);  // Ściana ukośna

  while (wb_robot_step(64) != -1) {
    // Możesz tutaj dodać logikę np. generowania labiryntu dynamicznie
  }

  wb_robot_cleanup();
  return 0;
}
