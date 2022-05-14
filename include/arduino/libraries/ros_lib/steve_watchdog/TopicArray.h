#ifndef _ROS_steve_watchdog_TopicArray_h
#define _ROS_steve_watchdog_TopicArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "steve_watchdog/TopicStatus.h"

namespace steve_watchdog
{

  class TopicArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t status_length;
      typedef steve_watchdog::TopicStatus _status_type;
      _status_type st_status;
      _status_type * status;

    TopicArray():
      header(),
      status_length(0), status(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->status_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->status_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->status_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->status_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->status_length);
      for( uint32_t i = 0; i < status_length; i++){
      offset += this->status[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t status_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      status_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->status_length);
      if(status_lengthT > status_length)
        this->status = (steve_watchdog::TopicStatus*)realloc(this->status, status_lengthT * sizeof(steve_watchdog::TopicStatus));
      status_length = status_lengthT;
      for( uint32_t i = 0; i < status_length; i++){
      offset += this->st_status.deserialize(inbuffer + offset);
        memcpy( &(this->status[i]), &(this->st_status), sizeof(steve_watchdog::TopicStatus));
      }
     return offset;
    }

    const char * getType(){ return "steve_watchdog/TopicArray"; };
    const char * getMD5(){ return "29f9ce4533fcf194256629d7d1c8d342"; };

  };

}
#endif
