#ifndef _ROS_sdc_msgs_arduinoIn_h
#define _ROS_sdc_msgs_arduinoIn_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sdc_msgs
{

  class arduinoIn : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint16_t analog[15];
      bool digital[53];

    arduinoIn():
      header(),
      analog(),
      digital()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      for( uint32_t i = 0; i < 15; i++){
      *(outbuffer + offset + 0) = (this->analog[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->analog[i] >> (8 * 1)) & 0xFF;
      offset += sizeof(this->analog[i]);
      }
      for( uint32_t i = 0; i < 53; i++){
      union {
        bool real;
        uint8_t base;
      } u_digitali;
      u_digitali.real = this->digital[i];
      *(outbuffer + offset + 0) = (u_digitali.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->digital[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      for( uint32_t i = 0; i < 15; i++){
      this->analog[i] =  ((uint16_t) (*(inbuffer + offset)));
      this->analog[i] |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->analog[i]);
      }
      for( uint32_t i = 0; i < 53; i++){
      union {
        bool real;
        uint8_t base;
      } u_digitali;
      u_digitali.base = 0;
      u_digitali.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->digital[i] = u_digitali.real;
      offset += sizeof(this->digital[i]);
      }
     return offset;
    }

    const char * getType(){ return "sdc_msgs/arduinoIn"; };
    const char * getMD5(){ return "0e859687178706c190dc9358e551329f"; };

  };

}
#endif