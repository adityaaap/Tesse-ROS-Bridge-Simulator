// Generated by gencpp from file tesse_ros_bridge/RepositionRequestServiceRequest.msg
// DO NOT EDIT!


#ifndef TESSE_ROS_BRIDGE_MESSAGE_REPOSITIONREQUESTSERVICEREQUEST_H
#define TESSE_ROS_BRIDGE_MESSAGE_REPOSITIONREQUESTSERVICEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace tesse_ros_bridge
{
template <class ContainerAllocator>
struct RepositionRequestServiceRequest_
{
  typedef RepositionRequestServiceRequest_<ContainerAllocator> Type;

  RepositionRequestServiceRequest_()
    : pose()  {
    }
  RepositionRequestServiceRequest_(const ContainerAllocator& _alloc)
    : pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
  _pose_type pose;





  typedef boost::shared_ptr< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RepositionRequestServiceRequest_

typedef ::tesse_ros_bridge::RepositionRequestServiceRequest_<std::allocator<void> > RepositionRequestServiceRequest;

typedef boost::shared_ptr< ::tesse_ros_bridge::RepositionRequestServiceRequest > RepositionRequestServiceRequestPtr;
typedef boost::shared_ptr< ::tesse_ros_bridge::RepositionRequestServiceRequest const> RepositionRequestServiceRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator1> & lhs, const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator2> & rhs)
{
  return lhs.pose == rhs.pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator1> & lhs, const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace tesse_ros_bridge

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f192399f711a48924df9a394d37edd67";
  }

  static const char* value(const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf192399f711a4892ULL;
  static const uint64_t static_value2 = 0x4df9a394d37edd67ULL;
};

template<class ContainerAllocator>
struct DataType< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "tesse_ros_bridge/RepositionRequestServiceRequest";
  }

  static const char* value(const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "## Reposition Request Service\n"
"\n"
"# Request fields\n"
"geometry_msgs/Pose pose\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RepositionRequestServiceRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::tesse_ros_bridge::RepositionRequestServiceRequest_<ContainerAllocator>& v)
  {
    s << indent << "pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // TESSE_ROS_BRIDGE_MESSAGE_REPOSITIONREQUESTSERVICEREQUEST_H
