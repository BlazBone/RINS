// Generated by gencpp from file my_package/Sum.msg
// DO NOT EDIT!


#ifndef MY_PACKAGE_MESSAGE_SUM_H
#define MY_PACKAGE_MESSAGE_SUM_H

#include <ros/service_traits.h>


#include <my_package/SumRequest.h>
#include <my_package/SumResponse.h>


namespace my_package
{

struct Sum
{

typedef SumRequest Request;
typedef SumResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Sum
} // namespace my_package


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::my_package::Sum > {
  static const char* value()
  {
    return "ea07415f981ad352cedb6941ee6c9fd6";
  }

  static const char* value(const ::my_package::Sum&) { return value(); }
};

template<>
struct DataType< ::my_package::Sum > {
  static const char* value()
  {
    return "my_package/Sum";
  }

  static const char* value(const ::my_package::Sum&) { return value(); }
};


// service_traits::MD5Sum< ::my_package::SumRequest> should match
// service_traits::MD5Sum< ::my_package::Sum >
template<>
struct MD5Sum< ::my_package::SumRequest>
{
  static const char* value()
  {
    return MD5Sum< ::my_package::Sum >::value();
  }
  static const char* value(const ::my_package::SumRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::my_package::SumRequest> should match
// service_traits::DataType< ::my_package::Sum >
template<>
struct DataType< ::my_package::SumRequest>
{
  static const char* value()
  {
    return DataType< ::my_package::Sum >::value();
  }
  static const char* value(const ::my_package::SumRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::my_package::SumResponse> should match
// service_traits::MD5Sum< ::my_package::Sum >
template<>
struct MD5Sum< ::my_package::SumResponse>
{
  static const char* value()
  {
    return MD5Sum< ::my_package::Sum >::value();
  }
  static const char* value(const ::my_package::SumResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::my_package::SumResponse> should match
// service_traits::DataType< ::my_package::Sum >
template<>
struct DataType< ::my_package::SumResponse>
{
  static const char* value()
  {
    return DataType< ::my_package::Sum >::value();
  }
  static const char* value(const ::my_package::SumResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MY_PACKAGE_MESSAGE_SUM_H
