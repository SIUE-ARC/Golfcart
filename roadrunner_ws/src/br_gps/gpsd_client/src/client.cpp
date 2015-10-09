#include <ros/ros.h>
#include <gps_common/GPSFix.h>
#include <gps_common/GPSStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <libgpsmm.h>
#include <stdio.h>
#include <iostream>

using namespace std;
using namespace gps_common;
using namespace sensor_msgs;

/////////////////////////////
// Class for the GPS class //
// This reads in the GPS   //
// Data and publishes      //
// The data.               //
/////////////////////////////
class GPSDClient {
  public:
    GPSDClient() : privnode("~"), gps(NULL), use_gps_time(true), check_fix_by_variance(true) {}
  
  /////////////////////////////
  // Initiates the variables //
  /////////////////////////////
    bool start() {
      // Sets up the publishing node /extended_fix
      gps_fix_pub = node.advertise<GPSFix>("extended_fix", 1);
      // Sets up the publishing node /fix
      navsat_fix_pub = node.advertise<NavSatFix>("fix", 1);

      // Sets up the time from the GPS
      privnode.getParam("use_gps_time", use_gps_time);
      // Sets up to check the variance of the GPS
      privnode.getParam("check_fix_by_variance", check_fix_by_variance);

      // Sets up the host to localhost
      std::string host = "localhost";
      //Initiates the port to 2947 (default port)
      int port = 2947;
      // Sets the host to localhost
      privnode.getParam("host", host);
      // sets the port to 2947
      privnode.getParam("port", port);

      // List of ports
      char port_s[12];
      //prints out the ports
      snprintf(port_s, 12, "%d", port);

      // sets resp to null
      gps_data_t *resp = NULL;

      // Sets GPS to host and port
      // Sets resp to view data 
      // Depending on version, depends on how its open and read
#if GPSD_API_MAJOR_VERSION >= 5
      gps = new gpsmm(host.c_str(), port_s);
      resp = gps->stream(WATCH_ENABLE);
#elif GPSD_API_MAJOR_VERSION == 4
      gps = new gpsmm();
      gps->open(host.c_str(), port_s);
      resp = gps->stream(WATCH_ENABLE);
#else
      gps = new gpsmm();
      resp = gps->open(host.c_str(), port_s);
      gps->query("w\n");
#endif
      // If resp is null, it failed to receive data
      if (resp == NULL) {
        ROS_ERROR("Failed to open GPSd");
        return false;
      }
      // GPS opened
      ROS_INFO("GPSd opened");
      return true;
    } 

  /////////////////////////////////
  // This function steps through //
  // the GPS and reads the data  //
  /////////////////////////////////
    void step() {
      // checks to see the version
      // reads in the data
#if GPSD_API_MAJOR_VERSION >= 5
      if (!gps->waiting(1e6))
        return;

      gps_data_t *p = gps->read();
#else
      gps_data_t *p = gps->poll();
#endif
      process_data(p);
    }
  ///////////////////////
  // Stops the reading //
  // of the GPS data   //
  ///////////////////////
    void stop() {
      // gpsmm doesn't have a close method? OK ...
    }

  private:
    ros::NodeHandle node;
    ros::NodeHandle privnode;
    ros::Publisher gps_fix_pub;
    ros::Publisher navsat_fix_pub;
    gpsmm *gps;

    bool use_gps_time;
    bool check_fix_by_variance;

  ////////////////////////
  // processes the data //
  // to the gps and the //
  // navsat             //
  ////////////////////////
    void process_data(struct gps_data_t* p) {
      if (p == NULL)
        return;

      if (!p->online)
        return;
      
      process_data_gps(p);
      process_data_navsat(p);
    }


#if GPSD_API_MAJOR_VERSION >= 4
#define SATS_VISIBLE p->satellites_visible
#elif GPSD_API_MAJOR_VERSION == 3
#define SATS_VISIBLE p->satellites
#else
#error "gpsd_client only supports gpsd API versions 3+"
#endif
  ////////////////////////////
  // Sets the GPS data      //
  // for publishing to /fix //
  ////////////////////////////
    void process_data_gps(struct gps_data_t* p) {
      ros::Time time = ros::Time::now();

      GPSFix fix;
      GPSStatus status;
      // Sets time stamp 
      status.header.stamp = time;
      fix.header.stamp = time;
      // Sets how many satellites are used
      status.satellites_used = p->satellites_used;
     
      status.satellite_used_prn.resize(status.satellites_used);
      for (int i = 0; i < status.satellites_used; ++i) {
        status.satellite_used_prn[i] = p->used[i];
      }

      status.satellites_visible = SATS_VISIBLE;
      // Check the satellites data
      status.satellite_visible_prn.resize(status.satellites_visible);
      status.satellite_visible_z.resize(status.satellites_visible);
      status.satellite_visible_azimuth.resize(status.satellites_visible);
      status.satellite_visible_snr.resize(status.satellites_visible);
      // For every satellite that is visible, it reads the data
      for (int i = 0; i < SATS_VISIBLE; ++i) {
        status.satellite_visible_prn[i] = p->PRN[i];
        status.satellite_visible_z[i] = p->elevation[i];
        status.satellite_visible_azimuth[i] = p->azimuth[i];
        status.satellite_visible_snr[i] = p->ss[i];
      }

      if ((p->status & STATUS_FIX) && !(check_fix_by_variance && isnan(p->fix.epx))) {
        status.status = 0; // FIXME: gpsmm puts its constants in the global
                           // namespace, so `GPSStatus::STATUS_FIX' is illegal.

        if (p->status & STATUS_DGPS_FIX)
          status.status |= 18; // same here

	// sets data to /fix
        fix.time = p->fix.time;
        fix.latitude = p->fix.latitude;
        fix.longitude = p->fix.longitude;
        fix.altitude = p->fix.altitude;
        fix.track = p->fix.track;
        fix.speed = p->fix.speed;
        fix.climb = p->fix.climb;

#if GPSD_API_MAJOR_VERSION > 3
        fix.pdop = p->dop.pdop;
        fix.hdop = p->dop.hdop;
        fix.vdop = p->dop.vdop;
        fix.tdop = p->dop.tdop;
        fix.gdop = p->dop.gdop;
#else
        fix.pdop = p->pdop;
        fix.hdop = p->hdop;
        fix.vdop = p->vdop;
        fix.tdop = p->tdop;
        fix.gdop = p->gdop;
#endif

        fix.err = p->epe;
        fix.err_vert = p->fix.epv;
        fix.err_track = p->fix.epd;
        fix.err_speed = p->fix.eps;
        fix.err_climb = p->fix.epc;
        fix.err_time = p->fix.ept;

        /* TODO: attitude */
      } else {
      	status.status = -1; // STATUS_NO_FIX
      }
      // Sets the status 
      // Valid or or Not
      fix.status = status;
      // Publishes /fix
      gps_fix_pub.publish(fix);
    }

  // Prepares and Publishes /extended_fix
    void process_data_navsat(struct gps_data_t* p) {
      NavSatFixPtr fix(new NavSatFix);

      /* TODO: Support SBAS and other GBAS. */

      if (use_gps_time && !isnan(p->fix.time))
        fix->header.stamp = ros::Time(p->fix.time);
      else
        fix->header.stamp = ros::Time::now();

      /* gpsmm pollutes the global namespace with STATUS_,
       * so we need to use the ROS message's integer values
       * for status.status
       */
      switch (p->status) {
        case STATUS_NO_FIX:
          fix->status.status = -1; // NavSatStatus::STATUS_NO_FIX;
          break;
        case STATUS_FIX:
          fix->status.status = 0; // NavSatStatus::STATUS_FIX;
          break;
        case STATUS_DGPS_FIX:
          fix->status.status = 2; // NavSatStatus::STATUS_GBAS_FIX;
          break;
      }

      fix->status.service = NavSatStatus::SERVICE_GPS;

      fix->latitude = p->fix.latitude;
      fix->longitude = p->fix.longitude;
      fix->altitude = p->fix.altitude;

      /* gpsd reports status=OK even when there is no current fix,
       * as long as there has been a fix previously. Throw out these
       * fake results, which have NaN variance
       */
      if (isnan(p->fix.epx) && check_fix_by_variance) {
        return;
      }

      fix->position_covariance[0] = p->fix.epx;
      fix->position_covariance[4] = p->fix.epy;
      fix->position_covariance[8] = p->fix.epv;

      fix->position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      //cout << fix->position_covariance[0] << "  Covariance" << endl;
      navsat_fix_pub.publish(fix);
    }
};

///////////////////////////
// Main, calls the class //
// and repeats untill    //
// ros is no longer OK   //
///////////////////////////
int main(int argc, char ** argv) {
  ros::init(argc, argv, "gpsd_client");

  GPSDClient client;

  if (!client.start())
    return -1;


  while(ros::ok()) {
    ros::spinOnce();
    client.step();
  }

  client.stop();
}
