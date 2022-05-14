#ifndef _ROS_zed_interfaces_BoundingBox2Di_h
#define _ROS_zed_interfaces_BoundingBox2Di_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "zed_interfaces/Keypoint2Di.h"

namespace zed_interfaces
{

  class BoundingBox2Di : public ros::Msg
  {
    public:
      zed_interfaces::Keypoint2Di corners[4];

    BoundingBox2Di():
      corners()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += this->corners[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 4; i++){
      offset += this->corners[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "zed_interfaces/BoundingBox2Di"; };
    const char * getMD5(){ return "48f8f53e85eafba49fb9648f6258d114"; };

  };

}
#endif
