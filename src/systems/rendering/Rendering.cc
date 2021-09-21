/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include "Rendering.hh"

#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <sdf/Sensor.hh>

#include <ignition/math/Helpers.hh>

#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include "ignition/gazebo/components/Atmosphere.hh"
#include "ignition/gazebo/components/Camera.hh"
#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/GpuLidar.hh"
#include "ignition/gazebo/components/Renderable.hh"
#include "ignition/gazebo/components/RenderEngineServerHeadless.hh"
#include "ignition/gazebo/components/RenderEngineServerPlugin.hh"
#include "ignition/gazebo/components/RgbdCamera.hh"
#include "ignition/gazebo/components/SameProcess.hh"
#include "ignition/gazebo/components/ThermalCamera.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/rendering/Events.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

// Private data class.
class ignition::gazebo::systems::RenderingPrivate
{
  /// \brief used to store whether rendering objects have been created.
  public: bool initialized = false;

  /// \brief Main rendering interface
  public: RenderUtil renderUtil;

  /// \brief rendering scene to be managed by the scene manager and used to
  /// generate sensor data
  public: rendering::ScenePtr scene;

  /// \brief Flag to indicate if worker threads are running
  public: std::atomic<bool> running { false };

  /// \brief Flag to signal if initialization should occur
  public: bool doInit { false };

  /// \brief Flag to signal if rendering update is needed
  public: bool updateAvailable { false };

  public: bool sameProcess { false };

  /// \brief Thread that rendering will occur in
  public: std::thread renderThread;

  /// \brief Mutex to protect rendering data
  public: std::mutex renderMutex;

  /// \brief Condition variable to signal rendering thread
  ///
  /// This variable is used to block/unblock operations in the rendering
  /// thread.  For a more detailed explanation on the flow refer to the
  /// documentation on RenderThread.
  public: std::condition_variable renderCv;

  /// \brief Connection to events::Stop event, used to stop thread
  public: ignition::common::ConnectionPtr stopConn;

  /// \brief Connection to events::Render event, used to render thread
  public: ignition::common::ConnectionPtr renderConn;

  /// \brief Update time for the next rendering iteration
  public: std::chrono::steady_clock::duration updateTime;

  /// \brief Pointer to the event manager
  public: EventManager *eventManager{nullptr};

  /// \brief Wait for initialization to happen
  private: void WaitForInit();

  /// \brief Run one rendering iteration
  private: void RunOnce();

  /// \brief Top level function for the rendering thread
  ///
  /// This function captures all of the behavior of the rendering thread.
  /// The behavior is captured in two phases: initialization and steady state.
  ///
  /// When the thread is first started, it waits on renderCv until the
  /// prerequisites for initialization are met, and the `doInit` flag is set.
  /// In order for initialization to proceed, rendering sensors must be
  /// available in the EntityComponentManager.
  ///
  /// When doInit is set, and renderCv is notified, initialization
  /// is performed (creating the render context and scene). During
  /// initialization, execution is blocked for the caller of PostUpdate.
  /// When initialization is complete, PostUpdate will be notified via
  /// renderCv and execution will continue.
  ///
  /// Once in steady state, a rendering operation is triggered by setting
  /// updateAvailable to true, and notifying via the renderCv.
  /// The rendering operation is done in `RunOnce`.
  ///
  /// The caller of PostUpdate will not be blocked if there is no
  /// rendering operation currently ongoing. Rendering will occur
  /// asyncronously.
  //
  /// The caller of PostUpdate will be blocked if there is a rendering
  /// operation currently ongoing, until that completes.
  private: void RenderThread();

  /// \brief Launch the rendering thread
  public: void Run();

  /// \brief Stop the rendering thread
  public: void Stop();
};

//////////////////////////////////////////////////
void RenderingPrivate::WaitForInit()
{
  while (!this->initialized && this->running)
  {
    igndbg << "Waiting for init" << std::endl;
    std::unique_lock<std::mutex> lock(this->renderMutex);
    // Wait to be ready for initialization or stopped running.
    // We need rendering sensors to be available to initialize.
    this->renderCv.wait(lock, [this]()
    {
      return this->doInit || !this->running;
    });

    if (this->doInit)
    {
      // Only initialize if there are rendering sensors
      igndbg << "Initializing render context" << std::endl;
      this->renderUtil.Init();
      this->scene = this->renderUtil.Scene();
      this->scene->SetCameraPassCountPerGpuFlush(6u);
      this->initialized = true;
    }

    this->updateAvailable = false;
    this->renderCv.notify_one();
  }
  igndbg << "Rendering Thread initialized" << std::endl;
}

//////////////////////////////////////////////////
void RenderingPrivate::RunOnce()
{
  std::unique_lock<std::mutex> lock(this->renderMutex);
  this->renderCv.wait(lock, [this]()
  {
    return !this->running || this->updateAvailable;
  });

  if (!this->running)
    return;

  if (!this->scene)
    return;

  IGN_PROFILE("SensorsPrivate::RunOnce");
  {
    IGN_PROFILE("RenderUtil::Update");
    this->renderUtil.Update();
  }

  {
    IGN_PROFILE("PreRender");
    this->eventManager->Emit<events::PreRender>();
    // Update the scene graph manually to improve performance
    // We only need to do this once per frame. It is important to call
    // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
    // so we don't waste cycles doing one scene graph update per sensor
    this->scene->PreRender();
  }

  {
    IGN_PROFILE("PostRender");
    // Update the scene graph manually to improve performance
    // We only need to do this once per frame It is important to call
    // sensors::RenderingSensor::SetManualSceneUpdate and set it to true
    // so we don't waste cycles doing one scene graph update per sensor
    this->scene->PostRender();
    this->eventManager->Emit<events::PostRender>();
  }

  this->updateAvailable = false;
  lock.unlock();
  this->renderCv.notify_one();
}

//////////////////////////////////////////////////
void RenderingPrivate::RenderThread()
{
  IGN_PROFILE_THREAD_NAME("RenderThread");

  // We have to wait for rendering sensors to be available
  this->WaitForInit();

  while (this->running)
  {
    this->RunOnce();
  }
}

//////////////////////////////////////////////////
void RenderingPrivate::Run()
{
  this->running = true;
  this->renderThread = std::thread(&RenderingPrivate::RenderThread, this);
}

//////////////////////////////////////////////////
void RenderingPrivate::Stop()
{
  std::unique_lock<std::mutex> lock(this->renderMutex);
  this->running = false;

  if (this->stopConn)
  {
    // Clear connection to stop additional incoming events.
    this->stopConn.reset();
  }

  lock.unlock();
  this->renderCv.notify_all();

  if (this->renderThread.joinable())
  {
    this->renderThread.join();
  }
}

//////////////////////////////////////////////////
Rendering::Rendering() : System(), dataPtr(std::make_unique<RenderingPrivate>())
{
}

//////////////////////////////////////////////////
Rendering::~Rendering()
{
  this->dataPtr->Stop();
}

//////////////////////////////////////////////////
void Rendering::Configure(const Entity &/*_id*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  igndbg << "Configuring Rendering system" << std::endl;
  // Setup rendering
  std::string engineName =
      _sdf->Get<std::string>("render_engine", "ogre2").first;

  this->dataPtr->renderUtil.SetEngineName(engineName);

  // parse sensor-specific data
  auto worldEntity = _ecm.EntityByComponents(components::World());
  if (kNullEntity != worldEntity)
  {
    // Set render engine if specified from command line
    auto renderEngineServerComp =
      _ecm.Component<components::RenderEngineServerPlugin>(worldEntity);
    if (renderEngineServerComp && !renderEngineServerComp->Data().empty())
    {
      this->dataPtr->renderUtil.SetEngineName(renderEngineServerComp->Data());
    }

    // Set headless mode if specified from command line
    auto renderEngineServerHeadlessComp =
      _ecm.Component<components::RenderEngineServerHeadless>(worldEntity);
    if (renderEngineServerHeadlessComp)
    {
      this->dataPtr->renderUtil.SetHeadlessRendering(
        renderEngineServerHeadlessComp->Data());
    }

    // Check if the server and gui are running in the same process
    auto sameProcessComp =
      _ecm.Component<components::SameProcess>(worldEntity);
    if (sameProcessComp)
    {
      this->dataPtr->sameProcess = sameProcessComp->Data();
    }
  }

  this->dataPtr->eventManager = &_eventMgr;

  this->dataPtr->stopConn = _eventMgr.Connect<events::Stop>(
      std::bind(&RenderingPrivate::Stop, this->dataPtr.get()));

  // Kick off worker thread
  this->dataPtr->Run();
}

//////////////////////////////////////////////////
void Rendering::Update(const UpdateInfo &_info,
                     EntityComponentManager &_ecm)
{
  IGN_PROFILE("Rendering::Update");
  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    this->dataPtr->renderUtil.UpdateECM(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void Rendering::PostUpdate(const UpdateInfo &_info,
                         const EntityComponentManager &_ecm)
{
  IGN_PROFILE("Rendering::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    ignwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized &&
      _ecm.HasComponentType(components::Renderable::typeId))
  {
    std::unique_lock<std::mutex> lock(this->dataPtr->renderMutex);
    igndbg << "Initialization needed" << std::endl;
    this->dataPtr->doInit = true;
    this->dataPtr->renderCv.notify_one();
  }

  if (this->dataPtr->running && this->dataPtr->initialized)
  {
    this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);
  }
}

IGNITION_ADD_PLUGIN(Rendering, System,
  Rendering::ISystemConfigure,
  Rendering::ISystemUpdate,
  Rendering::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(Rendering , "ignition::gazebo::systems::Rendering")
