/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef GZ_GAZEBO_COMPONENTS_LogPlaybackStatistics_HH_
#define GZ_GAZEBO_COMPONENTS_LogPlaybackStatistics_HH_

#include <ignition/msgs/log_playback_stats.pb.h>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/gazebo/Conversions.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component type that contains log playback,
  /// systems::LogPlayback, information.
  /// The log playback is created from world entity upon the playback plugin
  /// being loaded
  using LogPlaybackStatistics = Component<gz::msgs::LogPlaybackStatistics,
      class LogPlaybackStatisticsTag, serializers::MsgSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.LogPlaybackStatistics",
      LogPlaybackStatistics)
}
}
}
}

#endif
