// Generated by gencpp from file my_package/SumResponse.msg
// DO NOT EDIT!


#ifndef MY_PACKAGE_MESSAGE_SUMRESPONSE_H
#define MY_PACKAGE_MESSAGE_SUMRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace my_package
{
template <class ContainerAllocator>
struct SumResponse_
{
  typedef SumResponse_<ContainerAllocator> Type;

  SumResponse_()
    : sum(0)  {
    }
  SumResponse_(const ContainerAllocator& _alloc)
    : sum(0)  {
  (void)_alloc;
    }



   typedef int32_t _sum_type;
  _sum_type sum;





  typedef boost::shared_ptr< ::my_package::SumResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::my_package::SumResponse_<ContainerAllocator> const> ConstPtr;

}; // struct SumResponse_

typedef ::my_package::SumResponse_<std::allocator<void> > SumResponse;

typedef boost::shared_ptr< ::my_package::SumResponse > SumResponsePtr;
typedef boost::shared_ptr< ::my_package::SumResponse const> SumResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::my_package::SumResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::my_package::SumResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::my_package::SumResponse_<ContainerAllocator1> & lhs, const ::my_package::SumResponse_<ContainerAllocator2> & rhs)
{
  return lhs.sum == rhs.sum;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::my_package::SumResponse_<ContainerAllocator1> & lhs, const ::my_package::SumResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace my_package

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::my_package::SumResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::my_package::SumResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_package::SumResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::my_package::SumResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_package::SumResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::my_package::SumResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::my_package::SumResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0ba699c25c9418c0366f3595c0c8e8ec";
  }

  static const char* value(const ::my_package::SumResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0ba699c25c9418c0ULL;
  static const uint64_t static_value2 = 0x366f3595c0c8e8ecULL;
};

template<class ContainerAllocator>
struct DataType< ::my_package::SumResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "my_package/SumResponse";
  }

  static const char* value(const ::my_package::SumResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::my_package::SumResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 sum\n"
;
  }

  static const char* value(const ::my_package::SumResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::my_package::SumResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.sum);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SumResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::my_package::SumResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::my_package::SumResponse_<ContainerAllocator>& v)
  {
    s << indent << "sum: ";
    Printer<int32_t>::stream(s, indent + "  ", v.sum);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MY_PACKAGE_MESSAGE_SUMRESPONSE_H
