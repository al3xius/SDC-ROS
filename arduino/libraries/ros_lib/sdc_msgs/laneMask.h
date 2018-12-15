#ifndef _ROS_SERVICE_laneMask_h
#define _ROS_SERVICE_laneMask_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/Image.h"

namespace sdc_msgs
{

static const char LANEMASK[] = "sdc_msgs/laneMask";

  class laneMaskRequest : public ros::Msg
  {
    public:

    laneMaskRequest()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return LANEMASK; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class laneMaskResponse : public ros::Msg
  {
    public:
      typedef sensor_msgs::Image _mask_type;
      _mask_type mask;

    laneMaskResponse():
      mask()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->mask.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->mask.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return LANEMASK; };
    const char * getMD5(){ return "160e5d4ef269eecfbab15d8fec09aa58"; };

  };

  class laneMask {
    public:
    typedef laneMaskRequest Request;
    typedef laneMaskResponse Response;
  };

}
#endif
