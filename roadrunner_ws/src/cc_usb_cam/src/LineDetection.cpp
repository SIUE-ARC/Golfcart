// Cathy Casey
// LineDetection.cpp
// this file uses camera image information to detect lines for path traversal
// last modified 13 February 2015

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "LineDetection.h"

//make rgb an array?
string determineColor(int r, int g, int b)
{
  string color;
  // if the 
  // allows hints of green (from grass)
  if(r > MAGIC_COLOR && g > MAGIC_COLOR & b > MAGIC_COLOR)
  {
    color = "white";
  }
  else
  {
    color = "notWhite";
  }
  
  return color;
}

string detectLine(string camera)
{
  //handle the rgb value better
  getColor(r, g, b);
  string currColor = determineColor(r, g, b);
  string turnDirection;
  
  // this will be changed once stereoscopy is fully incorporated
  if(currColor == "white" && camera == "left" && camera != "right")
    {
      //turn right
      turnDirection = "right";
    }
  else if(currColor == "white" && camera == "right" && camera != "left")
    {
      //turn left
      turnDirection = "left";
    }
  else if(currColor == "white" && camera == "both")
    {
      //straight
      turnDirection = "straight";
    }
  else
    {
      //straight
      turnDirection = "straight";
    }
  return turnDirection;
}