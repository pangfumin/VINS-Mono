/*
* An analogue of ros std_msg::Header
* Create by Pang Fumin
*/
#ifndef UTILITY_HEADER_H_
#define UTILITY_HEADER_H_
#include "Time.hpp"
class Header{
public:
  Header(){};
    Header(double t):stamp(t),frameId(""){
    };
    Header(double t, int64_t systemTs):stamp(t),
                                       system_ts(systemTs),
                                       frameId(""){
    };

    Header(Time t):stamp(t),frameId(""){
    };
    Header(Time t, std::string frame_id):stamp(t),frameId(frame_id){};
    Header(const Header & other):stamp(other.stamp),
                                 frameId(other.frameId){};
    Header &operator=(const Header & other){
        if(&other == this)
            return *this;
        stamp = other.stamp;
        system_ts = other.system_ts;
        frameId = other.frameId;
        return *this;
    }
    Time stamp;
    int64_t system_ts;  // ninebot_slam_related [us]
    std::string frameId;
};
#endif  //  UTILITY_HEADER_H_