#include <iostream>
#include <librealsense2/rs.hpp>
#include <math.h>

struct vector3D {
  float x, y, z;
  vector3D add(float x, float y, float z) {
    return {this->x + x, this->y + y, this->z + z};
  }
  vector3D operator*(float dt) {
    return {this->x * dt, this->y * dt, this->z * dt};
  }
};

// theta is the angle of camera rotation in x, y and z components
vector3D theta;
std::mutex theta_mtx;
/* alpha indicates the part that gyro and accelerometer take in computation of
theta; higher alpha gives more weight to gyro, but too high values cause drift;
lower alpha gives more weight to accelerometer, which is more sensitive to
disturbances */
float alpha = 0.98;
bool first = true;
// Keeps the arrival time of previous gyro frame
double last_ts_gyro = 0;

double PI = 3.14159265;
double rad_to_deg = 180 / PI;

void process_gyro(vector3D data, double ts) {
  if (first) {
    last_ts_gyro = ts;
    return;
  }

  vector3D gyro_angle;
  gyro_angle.x = data.x;
  gyro_angle.y = data.y;
  gyro_angle.z = data.z;

  double dt = (ts - last_ts_gyro) / 1000.0;

  gyro_angle = gyro_angle * dt;

  theta.add(-gyro_angle.z, -gyro_angle.y, gyro_angle.x);
}

void process_accel(vector3D accel_data) {
  vector3D accel_angle;

  accel_angle.z = atan2(accel_data.y, accel_data.z);
  accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y +
                                           accel_data.z * accel_data.z));

  if (first) {
    first = false;
    theta = accel_angle;
    theta.y = 0.0;
  } else {
    theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
    theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
  }
}

int main(int argc, char **argv) {
  rs2::pipeline pipe;

  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_GYRO);
  cfg.enable_stream(RS2_STREAM_ACCEL);

  pipe.start(cfg);

  while (true) // Application still alive?
  {
    rs2::frameset frameset = pipe.wait_for_frames();

    // Find and retrieve IMU data
    if (rs2::motion_frame gyro_frame =
            frameset.first_or_default(RS2_STREAM_GYRO)) {
      rs2_vector gyro_sample = gyro_frame.get_motion_data();
      // std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", "
      //           << gyro_sample.z << std::endl;

      vector3D gyro_data{gyro_sample.x, gyro_sample.y, gyro_sample.z};
      process_gyro(gyro_data, gyro_frame.get_timestamp());
      //...
    }

    if (rs2::motion_frame accel_frame =
            frameset.first_or_default(RS2_STREAM_ACCEL)) {
      rs2_vector accel_sample = accel_frame.get_motion_data();
      // std::cout << "Accel:" << accel_sample.x << ", " << accel_sample.y << ",
      // "
      //           << accel_sample.z << std::endl;

      vector3D accel_data{accel_sample.x, accel_sample.y, accel_sample.z};
      process_accel(accel_data);
      //...
    }

    std::cout << "Theta: " << theta.x * rad_to_deg << ", "
              << theta.y * rad_to_deg << ", " << theta.z * rad_to_deg
              << std::endl;
  }
}