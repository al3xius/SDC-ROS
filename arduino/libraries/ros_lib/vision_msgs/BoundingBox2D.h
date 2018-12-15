#ifndef _ROS_vision_msgs_BoundingBox2D_h
#define _ROS_vision_msgs_BoundingBox2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose2D.h"

namespace vision_msgs
{

  class BoundingBox2D : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose2D _center_type;
      _center_type center;
      typedef float _size_x_type;
      _size_x_type size_x;
      typedef float _size_y_type;
      _size_y_type size_y;

    BoundingBox2D():
      center(),
      size_x(0),
      size_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->center.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->size_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->size_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->center.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->size_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->size_y));
     return offset;
    }

    const char * getType(){ return "vision_msgs/BoundingBox2D"; };
    const char * getMD5(){ return "9ab41e2a4c8627735e5091a9abd68b02"; };

  };

}
#endif