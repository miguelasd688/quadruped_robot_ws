#ifndef VECTORS_H
#define VECTORS_H

#include <Arduino.h>
#include <math.h>

struct AngleVector
{
  float tetta;
  float alpha;
  float gamma;
};

struct LegsAngle
{
  struct AngleVector FR;
  struct AngleVector FL;
  struct AngleVector BR;
  struct AngleVector BL;
};

struct CoordinatesVector
{
  float x;
  float y;
  float z;
};

static float deg2rad(float deg){
  return deg* 2 * PI / 360.0;
}
static float rad2deg(float rad){
  return rad* 360.0 / (2 * PI);
}

#endif