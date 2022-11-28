// Generated by gencpp from file mynteye_wrapper_d/GetParamsRequest.msg
// DO NOT EDIT!


#ifndef MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMSREQUEST_H
#define MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMSREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace mynteye_wrapper_d
{
template <class ContainerAllocator>
struct GetParamsRequest_
{
  typedef GetParamsRequest_<ContainerAllocator> Type;

  GetParamsRequest_()
    : key(0)  {
    }
  GetParamsRequest_(const ContainerAllocator& _alloc)
    : key(0)  {
  (void)_alloc;
    }



   typedef uint32_t _key_type;
  _key_type key;



// reducing the odds to have name collisions with Windows.h 
#if defined(_WIN32) && defined(IMG_INTRINSICS)
  #undef IMG_INTRINSICS
#endif
#if defined(_WIN32) && defined(IMG_EXTRINSICS_RTOL)
  #undef IMG_EXTRINSICS_RTOL
#endif
#if defined(_WIN32) && defined(IMU_INTRINSICS)
  #undef IMU_INTRINSICS
#endif
#if defined(_WIN32) && defined(IMU_EXTRINSICS)
  #undef IMU_EXTRINSICS
#endif

  enum {
    IMG_INTRINSICS = 0u,
    IMG_EXTRINSICS_RTOL = 1u,
    IMU_INTRINSICS = 2u,
    IMU_EXTRINSICS = 3u,
  };


  typedef boost::shared_ptr< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> const> ConstPtr;

}; // struct GetParamsRequest_

typedef ::mynteye_wrapper_d::GetParamsRequest_<std::allocator<void> > GetParamsRequest;

typedef boost::shared_ptr< ::mynteye_wrapper_d::GetParamsRequest > GetParamsRequestPtr;
typedef boost::shared_ptr< ::mynteye_wrapper_d::GetParamsRequest const> GetParamsRequestConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator1> & lhs, const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator2> & rhs)
{
  return lhs.key == rhs.key;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator1> & lhs, const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace mynteye_wrapper_d

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "e7693783193e81fc44c88804af723942";
  }

  static const char* value(const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xe7693783193e81fcULL;
  static const uint64_t static_value2 = 0x44c88804af723942ULL;
};

template<class ContainerAllocator>
struct DataType< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "mynteye_wrapper_d/GetParamsRequest";
  }

  static const char* value(const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 IMG_INTRINSICS=0\n"
"uint32 IMG_EXTRINSICS_RTOL=1\n"
"uint32 IMU_INTRINSICS=2\n"
"uint32 IMU_EXTRINSICS=3\n"
"uint32 key\n"
;
  }

  static const char* value(const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.key);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetParamsRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::mynteye_wrapper_d::GetParamsRequest_<ContainerAllocator>& v)
  {
    s << indent << "key: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.key);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMSREQUEST_H
