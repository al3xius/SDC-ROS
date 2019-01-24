#ifndef _ROS_sdc_msgs_state_h
#define _ROS_sdc_msgs_state_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace sdc_msgs
{

  class state : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _mode_type;
      _mode_type mode;
      typedef int32_t _batteryCharge_type;
      _batteryCharge_type batteryCharge;
      typedef int32_t _gasPedal_type;
      _gasPedal_type gasPedal;
      typedef int32_t _velocity_type;
      _velocity_type velocity;
      typedef int32_t _targetVelocity_type;
      _targetVelocity_type targetVelocity;
      typedef int32_t _throttle_type;
      _throttle_type throttle;
      typedef bool _enableMotor_type;
      _enableMotor_type enableMotor;
      typedef int8_t _direction_type;
      _direction_type direction;
      typedef int32_t _steeringAngle_type;
      _steeringAngle_type steeringAngle;
      typedef bool _enableSteering_type;
      _enableSteering_type enableSteering;
      typedef const char* _indicate_type;
      _indicate_type indicate;
      typedef bool _light_type;
      _light_type light;

    state():
      header(),
      mode(""),
      batteryCharge(0),
      gasPedal(0),
      velocity(0),
      targetVelocity(0),
      throttle(0),
      enableMotor(0),
      direction(0),
      steeringAngle(0),
      enableSteering(0),
      indicate(""),
      light(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_mode = strlen(this->mode);
      varToArr(outbuffer + offset, length_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->mode, length_mode);
      offset += length_mode;
      union {
        int32_t real;
        uint32_t base;
      } u_batteryCharge;
      u_batteryCharge.real = this->batteryCharge;
      *(outbuffer + offset + 0) = (u_batteryCharge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_batteryCharge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_batteryCharge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_batteryCharge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->batteryCharge);
      union {
        int32_t real;
        uint32_t base;
      } u_gasPedal;
      u_gasPedal.real = this->gasPedal;
      *(outbuffer + offset + 0) = (u_gasPedal.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gasPedal.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gasPedal.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gasPedal.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gasPedal);
      union {
        int32_t real;
        uint32_t base;
      } u_velocity;
      u_velocity.real = this->velocity;
      *(outbuffer + offset + 0) = (u_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity);
      union {
        int32_t real;
        uint32_t base;
      } u_targetVelocity;
      u_targetVelocity.real = this->targetVelocity;
      *(outbuffer + offset + 0) = (u_targetVelocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_targetVelocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_targetVelocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_targetVelocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->targetVelocity);
      union {
        int32_t real;
        uint32_t base;
      } u_throttle;
      u_throttle.real = this->throttle;
      *(outbuffer + offset + 0) = (u_throttle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle);
      union {
        bool real;
        uint8_t base;
      } u_enableMotor;
      u_enableMotor.real = this->enableMotor;
      *(outbuffer + offset + 0) = (u_enableMotor.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableMotor);
      union {
        int8_t real;
        uint8_t base;
      } u_direction;
      u_direction.real = this->direction;
      *(outbuffer + offset + 0) = (u_direction.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->direction);
      union {
        int32_t real;
        uint32_t base;
      } u_steeringAngle;
      u_steeringAngle.real = this->steeringAngle;
      *(outbuffer + offset + 0) = (u_steeringAngle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steeringAngle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steeringAngle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steeringAngle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steeringAngle);
      union {
        bool real;
        uint8_t base;
      } u_enableSteering;
      u_enableSteering.real = this->enableSteering;
      *(outbuffer + offset + 0) = (u_enableSteering.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enableSteering);
      uint32_t length_indicate = strlen(this->indicate);
      varToArr(outbuffer + offset, length_indicate);
      offset += 4;
      memcpy(outbuffer + offset, this->indicate, length_indicate);
      offset += length_indicate;
      union {
        bool real;
        uint8_t base;
      } u_light;
      u_light.real = this->light;
      *(outbuffer + offset + 0) = (u_light.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->light);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_mode;
      arrToVar(length_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_mode-1]=0;
      this->mode = (char *)(inbuffer + offset-1);
      offset += length_mode;
      union {
        int32_t real;
        uint32_t base;
      } u_batteryCharge;
      u_batteryCharge.base = 0;
      u_batteryCharge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_batteryCharge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_batteryCharge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_batteryCharge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->batteryCharge = u_batteryCharge.real;
      offset += sizeof(this->batteryCharge);
      union {
        int32_t real;
        uint32_t base;
      } u_gasPedal;
      u_gasPedal.base = 0;
      u_gasPedal.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gasPedal.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gasPedal.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gasPedal.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gasPedal = u_gasPedal.real;
      offset += sizeof(this->gasPedal);
      union {
        int32_t real;
        uint32_t base;
      } u_velocity;
      u_velocity.base = 0;
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->velocity = u_velocity.real;
      offset += sizeof(this->velocity);
      union {
        int32_t real;
        uint32_t base;
      } u_targetVelocity;
      u_targetVelocity.base = 0;
      u_targetVelocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_targetVelocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_targetVelocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_targetVelocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->targetVelocity = u_targetVelocity.real;
      offset += sizeof(this->targetVelocity);
      union {
        int32_t real;
        uint32_t base;
      } u_throttle;
      u_throttle.base = 0;
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle = u_throttle.real;
      offset += sizeof(this->throttle);
      union {
        bool real;
        uint8_t base;
      } u_enableMotor;
      u_enableMotor.base = 0;
      u_enableMotor.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableMotor = u_enableMotor.real;
      offset += sizeof(this->enableMotor);
      union {
        int8_t real;
        uint8_t base;
      } u_direction;
      u_direction.base = 0;
      u_direction.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->direction = u_direction.real;
      offset += sizeof(this->direction);
      union {
        int32_t real;
        uint32_t base;
      } u_steeringAngle;
      u_steeringAngle.base = 0;
      u_steeringAngle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steeringAngle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steeringAngle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steeringAngle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steeringAngle = u_steeringAngle.real;
      offset += sizeof(this->steeringAngle);
      union {
        bool real;
        uint8_t base;
      } u_enableSteering;
      u_enableSteering.base = 0;
      u_enableSteering.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enableSteering = u_enableSteering.real;
      offset += sizeof(this->enableSteering);
      uint32_t length_indicate;
      arrToVar(length_indicate, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_indicate; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_indicate-1]=0;
      this->indicate = (char *)(inbuffer + offset-1);
      offset += length_indicate;
      union {
        bool real;
        uint8_t base;
      } u_light;
      u_light.base = 0;
      u_light.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->light = u_light.real;
      offset += sizeof(this->light);
     return offset;
    }

    const char * getType(){ return "sdc_msgs/state"; };
    const char * getMD5(){ return "770769294d7c76bc1957d9b669282e2c"; };

  };

}
#endif