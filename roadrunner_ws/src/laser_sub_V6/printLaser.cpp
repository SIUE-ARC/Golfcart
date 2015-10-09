/* File: printLaser.cpp
   Author: Brian Stroub
   Updated: 3/29/2014

   Description: This program subscribes to the laser node and prints out
   the data. 
 */


#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "laser_sub/Num.h"
#include "laser_sub/lasArray.h"
#include <vector>
#include <string>
#include <stdlib.h>
#include <cmath>

using namespace std;

#define SIZE 20

#define LASER_RANGE 10.0
#define ROBOT_X 0.0
#define ROBOT_Y 0.0

vector<float> objArray;


struct Coordinate
{
public:
	float x;
	float y;
	bool f;

};


struct Point
{
	float xPoint;
	float yPoint;
	bool flag;


	Point* next;
	Point* prev;

	Point(float x, float y, bool f)
	{
		xPoint = x;
		yPoint = y;
		flag = f;
		next = NULL;
		prev = NULL;
	}
	Point(float x, float y, Point* n, Point* p,bool f)
	{
		xPoint = x;
		yPoint = y;
		flag = f;
		next = n;
		prev = p;
	}

};

class linkedList
{

public:
	Point *head;
	linkedList()
	{
		head = NULL;
	}
	void addPoint(float x, float y, bool f)
	{
		Point *temp;
		if(head == NULL)
		{
			head = new Point(x, y, NULL, NULL,false);
		}
		else
		{
			temp = head;
			while(temp->next != NULL)
			{
				temp->prev = temp;
				temp = temp->next;

			}
			temp->next = new Point(x, y, NULL, temp,false);
		}
		temp = NULL;
	}


	int size()
	{
		int count = 0;

		Point* iter = head;
		while (iter != NULL)
		{
			count++;
			iter = iter->next;
		}
		return count;
	}

	bool compare(struct Point *point1, struct Point *point2)
	{
		bool comp = false;
		//ROS_INFO("x1: %f ", point1->xPoint);
		//ROS_INFO("x2: %f ", point2->xPoint);
		//ROS_INFO("y1: %f ", point1->yPoint);
		//ROS_INFO("y2: %f ", point2->yPoint);
		float x = (point2->xPoint - point1->xPoint);
		float y = (point2->yPoint - point1->yPoint);
		float check = sqrt(pow(x,2)+pow(y,2));

		//ROS_INFO("Check Value = %f", check);
		//error check if they are both null

		if (point1 == NULL && point2 == NULL)
		{
			comp = true;
		}
		else
		{
			if (point1 == NULL || point2 == NULL)
				comp = false;
			else if (check <= 10 && check > 0)
				comp = true;
			else
				comp = false;
		}
		point1 = NULL;
		point2 = NULL;

		//cout << iter->xPoint << " " << iter->yPoint << "\n";
		return comp;
	}

	void print(string name)
	{
	  /*
		ofstream myfile;
		myfile.open (name);
		//myfile << "Writing this to a file.\n";

		Point *temp;
		temp = head;
		int i = 0;
		while(temp != NULL)
		{
			//ROS_INFO("xPoint: %f, yPoint: %f", temp->xPoint, temp->yPoint);
			myfile << i << ". " << temp->xPoint  << " " << temp->yPoint << " " << std::boolalpha << temp->flag << endl;
			temp = temp->next;
			i++;
		}
		temp = NULL;
		myfile.close();
	  */
	}


};


class Obstacle
{
public: 
	double length;   //determined by the distance between first point and last point (cilinder and box might be diffrent
	float startX;
	float startY;
	float endX;
	float endY;
	float minX;
	float minY;
	float CenterX;
	float CenterY;
	float radius;
	linkedList detectedPoints;

};


Obstacle createObstacle()
{
	Obstacle newObstacle;


	return newObstacle;
}



//Here is where the implementation of the pessage received function goes
//and prints to the screen. 
void scanMessageReceived(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	//std::cout <<"I am here" << std::endl;
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<laser_sub::lasArray>("array", 100);
	int countMe = 0;
	int obstacleCount = 0;
	linkedList obj;
	linkedList obstacles;
	// linkedList points;
	//std_msgs::Int32MultiArray detectedObj;
	vector<Obstacle> detectedObj;

	//Store the recent laser points that have been validated
	vector<int> laser_x;
	vector<int> laser_y;

	//Created number of obstacles that have been detected and increment this value each time.

	//initialize the closest_angle
	float closest_angle;

	//loop to search threw the scan data
	for (int i = 0; i < msg->ranges.size(); i++)
	{
		float currentRange = msg->ranges[i];

		closest_angle = ((float)i*msg->angle_increment) + msg->angle_min;
		float x = currentRange * cos(closest_angle);
		float y = currentRange * sin(closest_angle);
		//ROS_INFO("X: %f ", x);
		//ROS_INFO("Y: %f ", y);
		//cout << x << endl;
		//cout << y << endl;
		//cout << endl;

		float xCal = (x - ROBOT_X);
		float yCal = (y - ROBOT_Y);
		float dist = sqrt(pow(xCal,2)+pow(yCal,2));
		ROS_INFO("Distance from robot: %f ", dist);/////////////////////////////////////////////////////////////////////////////////////////

		//locate laser scan cells within a certain range
		if (currentRange < LASER_RANGE) 
		{

			laser_x.push_back(x);
			laser_y.push_back(y);

		}

	}

	// mark obstacles and remove from vector to add more objects
	int i = 0;
	vector<int>::iterator it = laser_x.begin();
	vector<int>::iterator ity = laser_y.begin();
	while (it != laser_x.end() && ity != laser_y.end()) 
	{
		obj.addPoint(laser_x[i], laser_y[i], false);
		it = laser_x.erase(it);  
		ity = laser_y.erase(ity);
	}

	/*string file = "before.txt";
	obj.print(file);*/
	//cout << "obj size: "<< obj.size() << endl;
	//ROS_INFO("End of obj list\n");


	//traverse threw the list to create filter obstacles
	Point *iter;
	iter = obj.head;
	int zeroCount = 0;

	while(iter != NULL)
	{
		bool isObst = false;

		if ((iter->xPoint < 1 && iter->xPoint > -1) && (iter->yPoint < 1 && iter->yPoint > -1))
		{
			iter->flag = true;
		}
		else {}


		iter = iter->next;


	}
	iter = NULL;



	iter = obj.head;


	while(iter != NULL)
	{
		if(iter->yPoint < 0)
		{
			iter->xPoint = 0;
			iter->yPoint = 0;
			iter->flag = true;
		}

		iter = iter->next;
	}
	iter = NULL;
	/*string file2 = "after.txt";
	obj.print(file2);*/
	//obstacles.print();
	/*ROS_INFO("End of obstacle list\n");
	obstacleCount = obstacles.size();
	ROS_INFO("Obstacle List Count: %d ", obstacleCount);
	*/

	//re-structure to grab the start points and then travers till the end points.  

	iter = obj.head;
	Obstacle newObstacle;
	while(iter != NULL && iter->next != NULL)
	{


		if (iter->flag == false)
		{
			//this is the first point of the object
			newObstacle = createObstacle();
			newObstacle.startX = iter->xPoint;
			newObstacle.startY = iter->yPoint;
			while (iter->flag == false && (iter != NULL && iter->next != NULL))
			{

				newObstacle.detectedPoints.addPoint(iter->xPoint, iter->yPoint, false);

				//cout << iter->prev->xPoint << " " << iter->prev->yPoint << endl;
				ROS_INFO("startX: %f, startY: %f", newObstacle.startX, newObstacle.startY);/////////////////////////////////////////


				if(iter->next->flag == true)
				{
					newObstacle.endX = iter->xPoint;
					newObstacle.endY = iter->yPoint;
					if(newObstacle.detectedPoints.size() < 10)
					{}
					else
					{
						detectedObj.push_back(newObstacle);
					}
				}
				iter = iter->next;
			} 
		}
		else
		{
			iter = iter->next;
		}


	}
	iter = NULL;


	/*int BarrelCount = detectedObj.size();
	ROS_INFO("Obstacle Count: %d", BarrelCount);
	cout << "Obstacle Count: " << BarrelCount << endl;
	*/
	//Travers through the detectedObj vector and calculate the center and the max value of the object. 
	for (int i = 0; i < detectedObj.size(); i++)
	{
		Point *iter;
		iter = detectedObj[i].detectedPoints.head;
		float minX = iter->xPoint;
		float minY = iter->yPoint;
		float xCal = (minX - ROBOT_X);
		float yCal = (minY - ROBOT_Y);
		float dist1 = sqrt(pow(xCal,2)+pow(yCal,2));

		while(iter != NULL)
		{
			float xCal = (minX - ROBOT_X);
			float yCal = (minY - ROBOT_Y);
			float dist2 = sqrt(pow(xCal,2)+pow(yCal,2));
			if(dist2 < dist1)
			{
				minX = iter->xPoint;
				minY = iter->yPoint;
			}
			iter = iter->next;
		}
		detectedObj[i].minX = minX;
		detectedObj[i].minY = minY;

		//Calculate the center of the object 

		//Heron's Formula for the area of the triangle
		float xCal0 = (detectedObj[i].endX - detectedObj[i].startX);
		float yCal0 = (detectedObj[i].endY - detectedObj[i].startY);
		float a = sqrt(pow(xCal,2)+pow(yCal,2));

		float xCal1 = (detectedObj[i].startX - detectedObj[i].minX);
		float yCal1 = (detectedObj[i].startY - detectedObj[i].minY);
		float b = sqrt(pow(xCal,2)+pow(yCal,2));

		float xCal2 = (detectedObj[i].minX - detectedObj[i].endX);
		float yCal2 = (detectedObj[i].minY - detectedObj[i].endY);
		float c = sqrt(pow(xCal,2)+pow(yCal,2));

		float s = (a + b + c)/2;

		float area = sqrt(s*(s - a)*(s - b)*(s - c));

		//Outer Radius of the circle that circumscribes the triangle/object
		float outerRadius = (a*b*c)/(4*area);

		detectedObj[i].radius = outerRadius;
		

		float centerX = (detectedObj[i].startX + detectedObj[i].minX + detectedObj[i].endX)/3;
		float centerY = (detectedObj[i].startY + detectedObj[i].minY + detectedObj[i].endY)/3;

		detectedObj[i].CenterX = centerX;
		detectedObj[i].CenterY = centerY;


		
	}

	int BarrelCount = detectedObj.size();
	//ROS_INFO("Obstacle Count: %d", BarrelCount);

	//ros::NodeHandle n;
	//ros::Publisher pub = n.advertise<laser_sub::lasArray>("array", 1);
	for (int i = 0; i < detectedObj.size(); i++)
	  {
	    //assign array a random number between 0 and 255.
	    int x = detectedObj[i].CenterX;
	    int y = detectedObj[i].CenterY;
	    int r = detectedObj[i].radius;
	    
	    objArray.push_back(x);
	    objArray.push_back(y);
	    objArray.push_back(r);
	  }
	
	ros::Duration(0.5).sleep();
	//ros::Publisher pub = n.advertise<std_msgs::LaserScan>("array",1);
	//ros::Publisher pub = nh.advertise<laser_sub::Num>("Data", 5);
	::laser_sub::Num values;
	::laser_sub::lasArray array;
	
	float x;
	float y;
	float r;
  	//std::cout << "Detectedobjects: " << detectedObj.size() << std::endl;
	//std::cout << "objArraySize: " << objArray.size() << std::endl;
	for (int i = 0; i < objArray.size(); i++)
	  {
	 
	    x = objArray[i];
	    y = objArray[i++];
	    r = objArray[i++];

	    values.centerX = x;
	    values.centerY = y;
	    values.radius = r;
	    
	    array.laserArray.push_back(values);
	  }
	
	pub.publish(array);
	for(int x = 0; x < array.laserArray.size() -1;++x)
	{
		std::cout << "I should have just published: (" << array.laserArray[x].centerX << ", " << array.laserArray[x].centerY << ")" << std::endl;
	}
	std::cout << "-----------------------------------------------------------------------------" << std::endl;
	
	array.laserArray.clear();
	objArray.clear();
}

 

int main(int argc, char **argv)
{
  //Initialize the ROS system and become a node
  ros::init(argc, argv, "subscribe_to_lms1xx");
  ros::NodeHandle nh;
  //Creation of the subscriber object
  ros::Subscriber sub = nh.subscribe("scan", 1000, scanMessageReceived);
  /*
  ros::Publisher pub = nh.advertise<laser_sub::lasArray>("array", 1);
  //ros::Publisher pub = nh.advertise<laser_sub::Num>("Data", 5);
  ::laser_sub::Num values;
  ::laser_sub::lasArray array;
  float x;
  float y;
  float r;
  
  for (int i = 0; i < objArray.size(); i++)
    {
	 
      x = objArray[i];
      //array.lArray.push_back(x);
      y = objArray[i + 1];
      //array.lArray.push_back(y);
      r = objArray[i + 1];
      //array.lArray.push_back(r);
      //ROS_INFO("X: %f , Y: %f , R: %f", x,y,r);
    }
  */
 
  /*
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<laser_sub::lasArray>("array", 1);
  pub.publish(array);
  */
  //Let ROS take over
  ros::spin();
  
}
