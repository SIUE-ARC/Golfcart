/* Author: Brian Stroub
Updated: 1/22/15
Description: This program is just a test of how using data from arrays can 
determine the objects being detected and then creating objects that the 
robot can not pass threw and will go arround using the vector field created. 

*/


#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <fstream>
#include <array>
#include <vector>
#include <cmath>
using namespace std;

vector<float> ranges;
//[541]

int obstCount = 0;
int dataArray();
int distanceArray();
int arrayLenght = 0;
float pgCounter = 0.0;
void createObst();
void garbage();

float angle_increment = 0.00872664619237;
float angle_min = -2.35619449615f;

#define SIZE 20
#define LASER_RANGE 15.0
#define ROBOT_X 0.0
#define ROBOT_Y 0.0

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

		cout << iter->xPoint << " " << iter->yPoint << "\n";
		return comp;
	}

	void print(string name)
	{
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
void scanMessageReceived(vector<float> ranges)
{
	int countMe = 0;
	int obstacleCount = 0;
	linkedList obj;
	linkedList obstacles;
	// linkedList points;
	vector<Obstacle> detectedObj;

	//Store the recent laser points that have been validated
	vector<int> laser_x;
	vector<int> laser_y;

	//Created number of obstacles that have been detected and increment this value each time.

	//initialize the closest_angle
	float closest_angle;

	//loop to search threw the scan data
	for (int i = 0; i < ranges.size(); i++)
	{
		float currentRange = ranges[i];

		closest_angle = ((float)i*angle_increment) + angle_min;
		float x = currentRange * cos(closest_angle);
		float y = currentRange * sin(closest_angle);
		//ROS_INFO("X: %f ", x);
		//ROS_INFO("Y: %f ", y);
		cout << x << endl;
		cout << y << endl;
		cout << endl;

		float xCal = (x - ROBOT_X);
		float yCal = (y - ROBOT_Y);
		float dist = sqrt(pow(xCal,2)+pow(yCal,2));
		//ROS_INFO("Distance from robot: %f ", dist);

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


		//isObst = obj.compare(iter, iter->next);
		////ROS_INFO("isObst: %s", isObst ? "true" : "false");

		//if (!isObst)
		//{

		//	iter->xPoint = 0;
		//	iter->yPoint = 0;
		//	obstacles.addPoint(iter->xPoint, iter->yPoint);
		//	obstacles.addPoint(iter->next->xPoint, iter->next->yPoint);

		//}
		//else
		//{
		//	obstacles.addPoint(iter->xPoint, iter->yPoint);
		//}



		//cout << "CountMe: " << countMe << endl;
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
			while (iter->flag == false)
			{

				newObstacle.detectedPoints.addPoint(iter->xPoint, iter->yPoint, false);

				//cout << iter->prev->xPoint << " " << iter->prev->yPoint << endl;
				//ROS_INFO("startX: %f, startY: %f", newObstacle.startX, newObstacle.startY);


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


	int BarrelCount = detectedObj.size();
	//ROS_INFO("Obstacle Count: %d", BarrelCount);
	//cout << "Obstacle Count: " << BarrelCount << endl;

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



}





int main(int argc, char **argv)
{
	//using namespace std;
	// ifstream is used for reading files
	// We'll read from a file called Sample.dat
	ifstream inf("1Barrel1Chair.txt");

	// If we couldn't open the input file stream for reading
	if (!inf)
	{
		// Print an error and exit
		cerr << "Uh oh, Sample.dat could not be opened for reading!" << endl;
		exit(1);
	}

	// While there's still stuff left to read
	while (inf)
	{
		// read stuff from the file into a string and print it
		std::string strInput;
		getline(inf, strInput, ',');
		double temp = ::atof(strInput.c_str());
		//cout << temp << endl;

		ranges.push_back((float)temp);
	}

	scanMessageReceived(ranges);
	return 0;
}
