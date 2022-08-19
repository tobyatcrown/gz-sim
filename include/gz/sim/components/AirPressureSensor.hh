/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_GAZEBO_COMPONENTS_AIRPRESSURE_HH_
#define GZ_GAZEBO_COMPONENTS_AIRPRESSURE_HH_

#include <sdf/Sensor.hh>

#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Serialization.hh>
#include <ignition/gazebo/config.hh>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
  /// \brief A component type that contains an air pressure sensor,
  /// sdf::AirPressure, information.
  using AirPressureSensor = Component<sdf::Sensor, class AirPressureSensorTag,
      serializers::SensorSerializer>;
  IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.AirPressureSensor",
      AirPressureSensor)
}
}
}
}

#endif
