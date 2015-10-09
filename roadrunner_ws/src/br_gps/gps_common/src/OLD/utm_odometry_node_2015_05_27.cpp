#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <string>
#include <math.h>
#include <stdlib.h>
#include <sound_play/sound_play.h>
#include <vector>
#include "../../../TurningForGPS/TurningForGPS/Agent.h"

using namespace gps_common;
using namespace std;

static ros::Publisher Drive_Motor;
const int MAX_POINTS = 5;
int GPSCounter = 0;
const float Variability = 0.00005;
string PsocTopic = "drive_tx";

// Sleep for the sounds
void sleepok(int t, ros::NodeHandle &nh)
{
  if (nh.ok())
		sleep(t);
}

//GPS List Structure
struct GPSList 
{
	bool visited;
	float longitude;
	float latitude;
	float smallLat;
	float smallLong;
	float largeLat;
	float largeLong;
};
// Vector for the GPS List Structure 
vector<GPSList> GPSPoints;

//Convert GPS Points into a large grid.
//Origin(0,0) our starting point.
//Every .0001 is a new grid point.
struct MassiveFuckingGrid
{
	int x;
	int y;
	double originX;
	double originY;
};

vector<MassiveFuckingGrid> GPSGrid;

//Declare Functions
std::pair< double, double> getHeading();
void SetGPSPoints();
double Turn( double Currlatitude,  double Currlongitude);
void InitializeAgent(AgentController*& _agents, double Currlatitude, double Currlongitude);
void GPS(const sensor_msgs::NavSatFixConstPtr& fix);
void ConvertToGrid(double longitude, double latitude);

int main(int argc, char **argv)
{
	SetGPSPoints();
	ros::init(argc,argv,"utm_odometry_node");
	ros::NodeHandle node;
	Drive_Motor = node.advertise<std_msgs::String>(PsocTopic,10);
	ros::Subscriber fix_sub = node.subscribe("gps/fix",10,GPS);
	ros::spin();
	return 0;
}

//Takes in a latitude and longitude and turns them into gridpoints on an x y grid (integers only)
void ConvertToGrid(double x, double y)
{
	
}

void SetGPSPoints()
{
	//First GPS Point
	GPSPoints.push_back(GPSList());
	GPSPoints[0].latitude = 38.791609098;
	GPSPoints[0].longitude = -90.000932301;
	
	//Second GPS Point
	GPSPoints.push_back(GPSList());
	GPSPoints[1].latitude = 38.791596436;
	GPSPoints[1].longitude = -90.000598514;

	//Third GPS Point
	GPSPoints.push_back(GPSList());
	GPSPoints[2].latitude = 38.79140798;
	GPSPoints[2].longitude = -90.00077116;

	//Fourth GPS Point
	GPSPoints.push_back(GPSList());
	GPSPoints[3].latitude = 38.79039807;
	GPSPoints[3].longitude = -90.000932298;
	
/*
	//Fifth GPS Point
	GPSPoints[4].latitude = ;
	GPSPoints[4].longitude = ;

	//Sixth GPS Point
	GPSPoints[5].latitude = ;
	GPSPoints[5].longitude = ;

	//Seventh GPS Point
	GPSPoints[6].latitude = ;
	GPSPoints[6].longitude = ;
*/
	GPSPoints.push_back(GPSList());
	for(int i = 0; i < MAX_POINTS; i++)
	{	
		//Small lat and longitude values
		GPSPoints[i].smallLat = GPSPoints[i].latitude - Variability;
		GPSPoints[i].smallLong = GPSPoints[i].longitude - Variability;
		//Large lat and longitude values
		GPSPoints[i].largeLat = GPSPoints[i].latitude + Variability;
		GPSPoints[i].largeLong = GPSPoints[i].longitude + Variability;
	}	
}

void GPS(const sensor_msgs::NavSatFixConstPtr& fix)
{
	//Play Sound
	ros::NodeHandle nh;
	sound_play::SoundClient sc;

	//Information sent to drive motor
	std_msgs::String GPSDriving;

	//Counter for initial data
	static int Count = 0;

	//The reading of Latitude and Longitude from /fix
	//Stored in variables to not have to make calls every time
	long double Latitude = fix->latitude;
        long double Longitude = fix->longitude;
	
	//This will set the Start latitude and longitude.
	if(Count < 2)
	{
		Count++;
		//Set up finishing longitude and latitude
		GPSPoints[MAX_POINTS-1].latitude = Latitude;
		GPSPoints[MAX_POINTS-1].longitude = Longitude;
		GPSPoints[MAX_POINTS-1].smallLat =  GPSPoints[MAX_POINTS-1].latitude - Variability;
		GPSPoints[MAX_POINTS-1].smallLong =  GPSPoints[MAX_POINTS-1].longitude - Variability;
		GPSPoints[MAX_POINTS-1].largeLat =  GPSPoints[MAX_POINTS-1].latitude + Variability;
		GPSPoints[MAX_POINTS-1].largeLong =  GPSPoints[MAX_POINTS-1].longitude + Variability;
		if(Count == 2){
			sc.playWave("/home/brad/Music/Hands.wav");
			sleepok(2,nh);
			sc.say("Go");
			sleepok(2,nh);
		}
	}


//////////////////
// Logic of GPS //
//////////////////
	//Go Forward
	if (!(Latitude > GPSPoints[GPSCounter].smallLat && Latitude < GPSPoints[GPSCounter].largeLat 
	   && Longitude > GPSPoints[GPSCounter].smallLong && Longitude < GPSPoints[GPSCounter].largeLong))
	{
		//sc.say("I Am Going Forward");
		//sleepok(2,nh);
		std::cout <<"Forward" << std::endl;
		GPSDriving.data = "F 105\r";
	}
	//Need to next point
	if (Latitude > GPSPoints[GPSCounter].smallLat && Latitude < GPSPoints[GPSCounter].largeLat 
	   && Longitude > GPSPoints[GPSCounter].smallLong && Longitude < GPSPoints[GPSCounter].largeLong)
	{
		sc.playWave("/home/brad/Music/BeepBeep.wav");
		sleepok(2,nh);
		sc.say("Turn");
		sleepok(2,nh);
		std::cout << "Turn" << std::endl;
		// Calculate turning
		long double Turn_Angle = Turn(Latitude,Longitude);
		ros::Duration(1.0).sleep();
		//slowly turn
		GPSDriving.data = "F 95\r";
		GPSDriving.data = "T " + boost::to_string(Turn_Angle) + "\r";
		//How long will this take?
		
		//Set point to visited
		GPSPoints[GPSCounter].visited = true;
	}	

	//Need to stop back at the start
	if((Latitude > GPSPoints[MAX_POINTS-1].smallLat && Latitude < GPSPoints[MAX_POINTS-1].largeLat 
	   && Longitude > GPSPoints[MAX_POINTS-1].smallLong && Longitude < GPSPoints[MAX_POINTS-1].largeLong)
	   && GPSPoints[MAX_POINTS-2].visited == true && GPSPoints[MAX_POINTS-1].visited == false)
	{ 
		// Stop
		sc.playWave("/home/brad/Music/Winning.mp3");
		sleepok(2,nh);
		std::cout <<"Stop" << std::endl;
		sc.say("Stop");
		sleepok(2,nh);
		GPSDriving.data = "F 0\r";
		GPSPoints[MAX_POINTS-1].visited = true;
	}

	//If we just marked the point as visited...We must go to next point
	if(GPSPoints[GPSCounter].visited == true)	{GPSCounter++;}

	//Send data to drive motor;
	Drive_Motor.publish(GPSDriving);
}

double Turn( double Currlatitude, double Currlongitude)
{
   double angle;
 // srand(static_cast<unsigned int>(time(NULL)));
 // AgentController* agent = NULL;
//  InitializeAgent(agent,Currlatitude,Currlongitude);
 // std::pair< double,  double> Heading = getHeading();
 // while(agent->Update());
	//agent->GetVelocity();
 // agent->SetGoal(GetEnd(GPSPoints[GPSCounter].latitude,GPSPoints[GPSCounter].longitude,Heading.second));

 // delete agent;
 // agent  = NULL;
 // return angle;
}

/*void InitializeAgent(AgentController*& _agent, double Currlatitude, double Currlongitude)
{
  std::pair< double,  double> Heading = getHeading();
  _agent = new AgentController;
  _agent->SetState(GetStart(Currlatitude,Currlongitude,Heading.first));
  _agent->SetGoal(GetEnd(GPSPoints[GPSCounter].latitude,GPSPoints[GPSCounter].longitude,Heading.second)); //End Position
}

//Returns the start and end heading (Where it will exit and enter the circles in radians)
std::pair < double,  double> getHeading()
{
 // ros::subscribe fix_two = node.subscribe("fix_two",10);
  return std::make_pair(0.00,PI);
}
*/
