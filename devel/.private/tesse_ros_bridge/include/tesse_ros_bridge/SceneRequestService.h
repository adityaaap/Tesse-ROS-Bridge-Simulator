// Generated by gencpp from file tesse_ros_bridge/SceneRequestService.msg
// DO NOT EDIT!


#ifndef TESSE_ROS_BRIDGE_MESSAGE_SCENEREQUESTSERVICE_H
#define TESSE_ROS_BRIDGE_MESSAGE_SCENEREQUESTSERVICE_H

#include <ros/service_traits.h>


#include <tesse_ros_bridge/SceneRequestServiceRequest.h>
#include <tesse_ros_bridge/SceneRequestServiceResponse.h>


namespace tesse_ros_bridge
{

struct SceneRequestService
{

typedef SceneRequestServiceRequest Request;
typedef SceneRequestServiceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SceneRequestService
} // namespace tesse_ros_bridge


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::tesse_ros_bridge::SceneRequestService > {
  static const char* value()
  {
    return "c185ea87dce4170cbb159dbc7c2d24cf";
  }

  static const char* value(const ::tesse_ros_bridge::SceneRequestService&) { return value(); }
};

template<>
struct DataType< ::tesse_ros_bridge::SceneRequestService > {
  static const char* value()
  {
    return "tesse_ros_bridge/SceneRequestService";
  }

  static const char* value(const ::tesse_ros_bridge::SceneRequestService&) { return value(); }
};


// service_traits::MD5Sum< ::tesse_ros_bridge::SceneRequestServiceRequest> should match
// service_traits::MD5Sum< ::tesse_ros_bridge::SceneRequestService >
template<>
struct MD5Sum< ::tesse_ros_bridge::SceneRequestServiceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::tesse_ros_bridge::SceneRequestService >::value();
  }
  static const char* value(const ::tesse_ros_bridge::SceneRequestServiceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::tesse_ros_bridge::SceneRequestServiceRequest> should match
// service_traits::DataType< ::tesse_ros_bridge::SceneRequestService >
template<>
struct DataType< ::tesse_ros_bridge::SceneRequestServiceRequest>
{
  static const char* value()
  {
    return DataType< ::tesse_ros_bridge::SceneRequestService >::value();
  }
  static const char* value(const ::tesse_ros_bridge::SceneRequestServiceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::tesse_ros_bridge::SceneRequestServiceResponse> should match
// service_traits::MD5Sum< ::tesse_ros_bridge::SceneRequestService >
template<>
struct MD5Sum< ::tesse_ros_bridge::SceneRequestServiceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::tesse_ros_bridge::SceneRequestService >::value();
  }
  static const char* value(const ::tesse_ros_bridge::SceneRequestServiceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::tesse_ros_bridge::SceneRequestServiceResponse> should match
// service_traits::DataType< ::tesse_ros_bridge::SceneRequestService >
template<>
struct DataType< ::tesse_ros_bridge::SceneRequestServiceResponse>
{
  static const char* value()
  {
    return DataType< ::tesse_ros_bridge::SceneRequestService >::value();
  }
  static const char* value(const ::tesse_ros_bridge::SceneRequestServiceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // TESSE_ROS_BRIDGE_MESSAGE_SCENEREQUESTSERVICE_H
