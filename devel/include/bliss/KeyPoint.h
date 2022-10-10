// Generated by gencpp from file bliss/KeyPoint.msg
// DO NOT EDIT!


#ifndef BLISS_MESSAGE_KEYPOINT_H
#define BLISS_MESSAGE_KEYPOINT_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <bliss/Point2f.h>

namespace bliss
{
template <class ContainerAllocator>
struct KeyPoint_
{
  typedef KeyPoint_<ContainerAllocator> Type;

  KeyPoint_()
    : pt()
    , size(0.0)
    , angle(0.0)
    , response(0.0)
    , octave(0)
    , class_id(0)  {
    }
  KeyPoint_(const ContainerAllocator& _alloc)
    : pt(_alloc)
    , size(0.0)
    , angle(0.0)
    , response(0.0)
    , octave(0)
    , class_id(0)  {
  (void)_alloc;
    }



   typedef  ::bliss::Point2f_<ContainerAllocator>  _pt_type;
  _pt_type pt;

   typedef float _size_type;
  _size_type size;

   typedef float _angle_type;
  _angle_type angle;

   typedef float _response_type;
  _response_type response;

   typedef int32_t _octave_type;
  _octave_type octave;

   typedef int32_t _class_id_type;
  _class_id_type class_id;





  typedef boost::shared_ptr< ::bliss::KeyPoint_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bliss::KeyPoint_<ContainerAllocator> const> ConstPtr;

}; // struct KeyPoint_

typedef ::bliss::KeyPoint_<std::allocator<void> > KeyPoint;

typedef boost::shared_ptr< ::bliss::KeyPoint > KeyPointPtr;
typedef boost::shared_ptr< ::bliss::KeyPoint const> KeyPointConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bliss::KeyPoint_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bliss::KeyPoint_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bliss::KeyPoint_<ContainerAllocator1> & lhs, const ::bliss::KeyPoint_<ContainerAllocator2> & rhs)
{
  return lhs.pt == rhs.pt &&
    lhs.size == rhs.size &&
    lhs.angle == rhs.angle &&
    lhs.response == rhs.response &&
    lhs.octave == rhs.octave &&
    lhs.class_id == rhs.class_id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bliss::KeyPoint_<ContainerAllocator1> & lhs, const ::bliss::KeyPoint_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bliss

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::bliss::KeyPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bliss::KeyPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bliss::KeyPoint_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bliss::KeyPoint_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bliss::KeyPoint_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bliss::KeyPoint_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bliss::KeyPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "11cefb08bec6034bef3e32ec473dc6a7";
  }

  static const char* value(const ::bliss::KeyPoint_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x11cefb08bec6034bULL;
  static const uint64_t static_value2 = 0xef3e32ec473dc6a7ULL;
};

template<class ContainerAllocator>
struct DataType< ::bliss::KeyPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bliss/KeyPoint";
  }

  static const char* value(const ::bliss::KeyPoint_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bliss::KeyPoint_<ContainerAllocator> >
{
  static const char* value()
  {
    return "#class cv::KeyPoint\n"
"#{\n"
"#    Point2f pt;\n"
"#    float size;\n"
"#    float angle;\n"
"#    float response;\n"
"#    int octave;\n"
"#    int class_id;\n"
"#}\n"
"\n"
"Point2f pt\n"
"float32 size\n"
"float32 angle\n"
"float32 response\n"
"int32 octave\n"
"int32 class_id\n"
"================================================================================\n"
"MSG: bliss/Point2f\n"
"#class cv::Point2f\n"
"#{\n"
"#    float x;\n"
"#    float y;\n"
"#}\n"
"\n"
"float32 x\n"
"float32 y\n"
;
  }

  static const char* value(const ::bliss::KeyPoint_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bliss::KeyPoint_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pt);
      stream.next(m.size);
      stream.next(m.angle);
      stream.next(m.response);
      stream.next(m.octave);
      stream.next(m.class_id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct KeyPoint_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bliss::KeyPoint_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bliss::KeyPoint_<ContainerAllocator>& v)
  {
    s << indent << "pt: ";
    s << std::endl;
    Printer< ::bliss::Point2f_<ContainerAllocator> >::stream(s, indent + "  ", v.pt);
    s << indent << "size: ";
    Printer<float>::stream(s, indent + "  ", v.size);
    s << indent << "angle: ";
    Printer<float>::stream(s, indent + "  ", v.angle);
    s << indent << "response: ";
    Printer<float>::stream(s, indent + "  ", v.response);
    s << indent << "octave: ";
    Printer<int32_t>::stream(s, indent + "  ", v.octave);
    s << indent << "class_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.class_id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BLISS_MESSAGE_KEYPOINT_H
