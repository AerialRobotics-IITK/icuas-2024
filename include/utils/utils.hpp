/*defining ROS stream color*/

namespace util
{
  enum PRINT_COLOR
  {
    BLACK,
    RED,
    GREEN,
    YELLOW,
    BLUE,
    MAGENTA,
    CYAN,
    WHITE,
    ENDCOLOR
  };

  static std::ostream& operator<<(std::ostream& os, PRINT_COLOR c)
  {
    switch(c)
    {
      case BLACK    : os << "\033[1;30m"; break;
      case RED      : os << "\033[1;31m"; break;
      case GREEN    : os << "\033[1;32m"; break;
      case YELLOW   : os << "\033[1;33m"; break;
      case BLUE     : os << "\033[1;34m"; break;
      case MAGENTA  : os << "\033[1;35m"; break;
      case CYAN     : os << "\033[1;36m"; break;
      case WHITE    : os << "\033[1;37m"; break;
      case ENDCOLOR : os << "\033[0m";    break;
      default       : os << "\033[1;37m";
    }
    return os;
  }
} //namespace util 

#define ROS_BLACK_STREAM(x)   ROS_INFO_STREAM(util::BLACK   << x << util::ENDCOLOR)
#define ROS_RED_STREAM(x)     ROS_INFO_STREAM(util::RED     << x << util::ENDCOLOR)
#define ROS_GREEN_STREAM(x)   ROS_INFO_STREAM(util::GREEN   << x << util::ENDCOLOR)
#define ROS_YELLOW_STREAM(x)  ROS_INFO_STREAM(util::YELLOW  << x << util::ENDCOLOR)
#define ROS_BLUE_STREAM(x)    ROS_INFO_STREAM(util::BLUE    << x << util::ENDCOLOR)
#define ROS_MAGENTA_STREAM(x) ROS_INFO_STREAM(util::MAGENTA << x << util::ENDCOLOR)
#define ROS_CYAN_STREAM(x)    ROS_INFO_STREAM(util::CYAN    << x << util::ENDCOLOR)

#define ROS_BLACK_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, util::BLACK   << x << util::ENDCOLOR)
#define ROS_RED_STREAM_COND(c, x)     ROS_INFO_STREAM_COND(c, util::RED     << x << util::ENDCOLOR)
#define ROS_GREEN_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, util::GREEN   << x << util::ENDCOLOR)
#define ROS_YELLOW_STREAM_COND(c, x)  ROS_INFO_STREAM_COND(c, util::YELLOW  << x << util::ENDCOLOR)
#define ROS_BLUE_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, util::BLUE    << x << util::ENDCOLOR)
#define ROS_MAGENTA_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, util::MAGENTA << x << util::ENDCOLOR)
#define ROS_CYAN_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, util::CYAN    << x << util::ENDCOLOR)
