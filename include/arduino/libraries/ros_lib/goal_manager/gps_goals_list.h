#ifndef _ROS_goal_manager_gps_goals_list_h
#define _ROS_goal_manager_gps_goals_list_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatFix.h"

namespace goal_manager
{

  class gps_goals_list : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t gpsgoals_length;
      typedef sensor_msgs::NavSatFix _gpsgoals_type;
      _gpsgoals_type st_gpsgoals;
      _gpsgoals_type * gpsgoals;

    gps_goals_list():
      header(),
      gpsgoals_length(0), gpsgoals(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->gpsgoals_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->gpsgoals_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->gpsgoals_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->gpsgoals_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gpsgoals_length);
      for( uint32_t i = 0; i < gpsgoals_length; i++){
      offset += this->gpsgoals[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t gpsgoals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      gpsgoals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      gpsgoals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      gpsgoals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->gpsgoals_length);
      if(gpsgoals_lengthT > gpsgoals_length)
        this->gpsgoals = (sensor_msgs::NavSatFix*)realloc(this->gpsgoals, gpsgoals_lengthT * sizeof(sensor_msgs::NavSatFix));
      gpsgoals_length = gpsgoals_lengthT;
      for( uint32_t i = 0; i < gpsgoals_length; i++){
      offset += this->st_gpsgoals.deserialize(inbuffer + offset);
        memcpy( &(this->gpsgoals[i]), &(this->st_gpsgoals), sizeof(sensor_msgs::NavSatFix));
      }
     return offset;
    }

    const char * getType(){ return "goal_manager/gps_goals_list"; };
    const char * getMD5(){ return "912774cd1b5543ea18c255186a871e32"; };

  };

}
#endif
