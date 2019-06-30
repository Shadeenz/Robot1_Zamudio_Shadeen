/*
 * File:          robot1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

#include <stdio.h>
#include <math.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define PI 3.14159


#define OBSTACLE_DIST 20.0


/*variables*/
enum {
  GO,
  TURN,
  FREEWAY,
  OBSTACLE
};
double straightLineAngle;


/*Auxiliar functions*/
int searchForObstacles(WbDeviceTag distance_sensor) {
  double distance_of_sensor = wb_distance_sensor_get_value(distance_sensor);
  printf("Distance%lf\n", distance_of_sensor );
  if (distance_of_sensor > OBSTACLE_DIST)
    return FREEWAY;
  else
    return OBSTACLE;
}

void advanceStraightLineRobot(WbDeviceTag *wheels, double velocity) {
  wb_motor_set_velocity(wheels[0], -velocity);
  wb_motor_set_velocity(wheels[1], -velocity);
}

void stopWheels(WbDeviceTag *wheels) {
  wb_motor_set_velocity(wheels[0], 0);
  wb_motor_set_velocity(wheels[1], 0);
}

void wheelsTurnRight(WbDeviceTag *wheels) {
  //double initial_position = wb_position_sensor_get_value()
  wb_motor_set_velocity(wheels[0], -6.66);
  wb_motor_set_velocity(wheels[1], 6.66);
}

double getAngleRobot(WbDeviceTag pos_sensor) {
  printf("Angle calculation\n");
  double angle, rotationAngleW1;

  rotationAngleW1 = wb_position_sensor_get_value(pos_sensor);
  angle = fabs(rotationAngleW1- straightLineAngle);
  printf("Angle: %lf\n", angle);

  return angle;
}

float clearAngleRobot() {
  printf("Clearing angle\n");
}

int main(int argc, char **argv)
{
  /* necessary to initialize webots stuff */
  wb_robot_init();

   //Motor devices
   WbDeviceTag wheel_right= wb_robot_get_device("motor_right");
   WbDeviceTag wheel_left= wb_robot_get_device("motor_left");

   WbDeviceTag wheels[2];
   wheels[0] = wheel_right;
   wheels[1] = wheel_left;

   wb_motor_set_position(wheel_right, INFINITY);
   wb_motor_set_position(wheel_left, INFINITY);

   float velocity = 6.66;
   int robot_state = GO;

   //distance sensor devices
   WbDeviceTag dist_sensor=wb_robot_get_device("distance_sensor");
   wb_distance_sensor_enable(dist_sensor, TIME_STEP);
   double distance_value;

  //encoder device
   WbDeviceTag encoder = wb_robot_get_device("encoder1");
   wb_position_sensor_enable(encoder, TIME_STEP);

   float angle;

  /* main loop*/
  while (wb_robot_step(TIME_STEP) != -1) {

    if (robot_state == GO) {
      distance_value = searchForObstacles(dist_sensor);

      if (distance_value== FREEWAY) {
        advanceStraightLineRobot(wheels, velocity);
        angle = wb_position_sensor_get_value(encoder);
        printf("Angle: %lf\n", angle);
      } else if (distance_value== OBSTACLE) {
        robot_state = TURN;
        stopWheels(wheels);
        straightLineAngle = wb_position_sensor_get_value(encoder);
      }
    } else if (robot_state == TURN) {
      wheelsTurnRight(wheels);
      angle = getAngleRobot(encoder);

      if (angle >= 0.275*4*PI) {
        robot_state = GO;
        stopWheels(wheels);
        clearAngleRobot();
      }
    }
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
