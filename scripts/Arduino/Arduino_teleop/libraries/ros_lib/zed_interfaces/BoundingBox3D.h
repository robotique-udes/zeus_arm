#ifndef _ROS_zed_interfaces_BoundingBox3D_h
#define _ROS_zed_interfaces_BoundingBox3D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "zed_interfaces/Keypoint3D.h"

namespace zed_interfaces
{

  class BoundingBox3D : public ros::Msg
  {
    public:
      zed_interfaces::Keypoint3D corners[8];

    BoundingBox3D():
      corners()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      offset += this->corners[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 8; i++){
      offset += this->corners[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    const char * getType(){ return "zed_interfaces/BoundingBox3D"; };
    const char * getMD5(){ return "3750b81144ba1b26f88ce23eeb4efd34"; };

  };

}
#endif
