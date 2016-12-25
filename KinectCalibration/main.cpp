#include "KinectCalibration.h"

int main(int argc, char *argv[])
{
  Kinect kinect;
  if (!kinect.initialize(3000.0)) {
    return -1;
  }

  kinect.colorAndDepthImage();

  return 0;
}