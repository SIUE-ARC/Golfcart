// Cathy Casey
// LineDetection.h
// this file uses camera image information to detect lines for path traversal
// last modified 13 February 2015

#ifndef LINE_DETECTION_H
#define LINE_DETECTION_H
#include "ros/ros.h"
#include "std_msgs/String.h"

class LineDetection
{
 public:
  string determineColor(int r, int g, int b);
  void detectLine(string camera);
};