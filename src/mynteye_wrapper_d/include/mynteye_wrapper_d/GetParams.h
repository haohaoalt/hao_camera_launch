// Generated by gencpp from file mynteye_wrapper_d/GetParams.msg
// DO NOT EDIT!


#ifndef MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMS_H
#define MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMS_H

#include <ros/service_traits.h>


#include <mynteye_wrapper_d/GetParamsRequest.h>
#include <mynteye_wrapper_d/GetParamsResponse.h>


namespace mynteye_wrapper_d
{

struct GetParams
{

typedef GetParamsRequest Request;
typedef GetParamsResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct GetParams
} // namespace mynteye_wrapper_d


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::mynteye_wrapper_d::GetParams > {
  static const char* value()
  {
    return "27490e4b5cf4d32a761bad9cafd48f69";
  }

  static const char* value(const ::mynteye_wrapper_d::GetParams&) { return value(); }
};

template<>
struct DataType< ::mynteye_wrapper_d::GetParams > {
  static const char* value()
  {
    return "mynteye_wrapper_d/GetParams";
  }

  static const char* value(const ::mynteye_wrapper_d::GetParams&) { return value(); }
};


// service_traits::MD5Sum< ::mynteye_wrapper_d::GetParamsRequest> should match
// service_traits::MD5Sum< ::mynteye_wrapper_d::GetParams >
template<>
struct MD5Sum< ::mynteye_wrapper_d::GetParamsRequest>
{
  static const char* value()
  {
    return MD5Sum< ::mynteye_wrapper_d::GetParams >::value();
  }
  static const char* value(const ::mynteye_wrapper_d::GetParamsRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::mynteye_wrapper_d::GetParamsRequest> should match
// service_traits::DataType< ::mynteye_wrapper_d::GetParams >
template<>
struct DataType< ::mynteye_wrapper_d::GetParamsRequest>
{
  static const char* value()
  {
    return DataType< ::mynteye_wrapper_d::GetParams >::value();
  }
  static const char* value(const ::mynteye_wrapper_d::GetParamsRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::mynteye_wrapper_d::GetParamsResponse> should match
// service_traits::MD5Sum< ::mynteye_wrapper_d::GetParams >
template<>
struct MD5Sum< ::mynteye_wrapper_d::GetParamsResponse>
{
  static const char* value()
  {
    return MD5Sum< ::mynteye_wrapper_d::GetParams >::value();
  }
  static const char* value(const ::mynteye_wrapper_d::GetParamsResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::mynteye_wrapper_d::GetParamsResponse> should match
// service_traits::DataType< ::mynteye_wrapper_d::GetParams >
template<>
struct DataType< ::mynteye_wrapper_d::GetParamsResponse>
{
  static const char* value()
  {
    return DataType< ::mynteye_wrapper_d::GetParams >::value();
  }
  static const char* value(const ::mynteye_wrapper_d::GetParamsResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MYNTEYE_WRAPPER_D_MESSAGE_GETPARAMS_H
