#pragma once

#include <vector>
#include <cstring>

#include "MainLoopTypes.hpp"

class TelemetryLoggingInterface {
 public:
  /**
   * @brief TelemetryLoggingInterface is a helper to conveniently log data
   * over telemetry without having to think output which outputs are already
   * occupied
   * @param maxNumberOfChannels the maximum number of channels available for
   * logging
   * @param out a reference to the output that is to be used for logging
   */
  TelemetryLoggingInterface(const int maxNumberOfChannels, MainLoopOutput& out)
    : maxNumberOfChannels_(maxNumberOfChannels), out_(out), channelInfo_(){
    }
  /**
   * @brief getChannelInfo a getter for the channelInfo_ to figure out, which
   * cahnnel contains which information
   * @return a constant reference to the channelInfo_ vector
   */
  const std::vector<const char*> getChannelInfo() { return channelInfo_; }
  /**
   * @log sens a vector via telemtry if there are still enough logging slots available
   */
  bool log(const Vec3f& data, const char* info)
  {
    // check if we still have enough channels to log this
    if (!(maxNumberOfChannels_ - nextChannel_ >= 3))
    {
      return false;
    }

    log(data.x, info);
    log(data.y, info);
    log(data.z, info);

    return true;
  }
  /**
   * @log sends a single scalar via telemetry if there is still a logging
   * slot available
   * @return true if logging was succesfull (e.g. there was stil a slot
   * available)
   */
  bool log(const float data, const char* info) {
    // check whether there is still a channel available or we didn't get a
    // reference to any output
    if (nextChannel_ >= maxNumberOfChannels_) {
      return false;
    }
    // log the data and increment the counter
    out_.telemetryOutputs_plusMinus100[nextChannel_++] = data;
    // add the info the the channel info so that we can later figure out, what
    // the channel contains
    channelInfo_.push_back(info);

    return true;
  }

 private:
  /// a counter that keeps track of the number of channels already used.
  int nextChannel_ = 0;
  /// the maximum number of channels available
  const int maxNumberOfChannels_ = 12;
  /// a reference to the output we should log to
  MainLoopOutput& out_;
  /// a vector of strings containting information about every channel
  std::vector<const char*> channelInfo_;
};
