#ifndef _ROS_zed_interfaces_BoundingBox2Df_h
#define _ROS_zed_interfaces_BoundingBox2Df_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "zed_interfaces/Keypoint2Df.h"

namespace zed_interfaces
{

  class BoundingBox2Df : public ros::Msg
  {
    public:
      zed_interfaces::Keypoint2Df corners[4];

    BoundingBox2Df():
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

    const char * getType(){ return "zed_interfaces/BoundingBox2Df"; };
    const char * getMD5(){ return "8ce1e9ea2b267e0dce506c975b1391b0"; };

  };

}
#endif
