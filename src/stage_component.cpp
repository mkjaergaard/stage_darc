/*
 * Copyright (c) 2012, Prevas A/S
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Prevas A/S nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * DARC Component wrapper for Stage Robot Simulator
 *
 * \author Morten Kjaergaard
 */

#include <boost/filesystem.hpp>
#include <stage.hh>
#include <darc/component.h>
#include <stage_darc/stage_robot.h>

class StageComponent : public darc::Component
{
protected:
  typedef std::vector<stage_darc::StageRobot*> RobotListType;
  RobotListType robots_;
  Stg::World* world_;
  bool init_done_;
  boost::mutex init_mutex_;
  boost::condition_variable init_condition_;

public:
  StageComponent(const std::string& instance_name, darc::Node::Ptr node);
  ~StageComponent() {};

  void runStageThread();

  void update()
  {
    for( RobotListType::iterator it = robots_.begin(); it != robots_.end(); it++ )
    {
      (*it)->update();
    }
  }

};

bool updateCallback(Stg::World* world, StageComponent * component)
{
  component->update();
  return false;
}

void StageComponent::runStageThread()
{
  const std::string file = "/home/local/darcbuild/stage_darc/world/simple.world";

  if ( !boost::filesystem::exists(file) )
  {
    std::cout << "Can't find world file" << std::endl;
    exit(1);
  }

  int argc = 0;
  char ** argv = 0;
  Stg::Init(&argc, &argv );

  world_ = new Stg::WorldGui(800, 700, "Stage");

  world_->Load(file);
  world_->AddUpdateCallback((Stg::world_callback_t)updateCallback, this);

  // Find the robots/sensors
  std::vector<Stg::Model*> models1 = world_->GetChildren();
  for (std::vector<Stg::Model*>::iterator it = models1.begin(); it != models1.end(); it++)
  {
    Stg::Model* mod = (*it);
    if( mod->GetModelType() == "position" )
    {
      std::cout << "Found Robot " << mod->TokenStr() << std::endl;

      std::vector<Stg::Model*> models2 = mod->GetChildren();
      for (std::vector<Stg::Model*>::iterator it = models2.begin(); it != models2.end(); it++)
      {
	Stg::Model* mod2 = (*it);
	if( mod2->GetModelType() == "ranger" )
	{
	  std::cout << "  with sensor " << mod2->TokenStr() << std::endl;
	  // Subscriber
	  robots_.push_back( new stage_darc::StageRobot( this, (Stg::ModelPosition*)mod, (Stg::ModelRanger*) mod2 ) );
	}
      }
    }
  }

  // Notify Component Thread that init is done
  {
    boost::lock_guard<boost::mutex> lock(init_mutex_);
    init_done_ = true;
  }
  init_condition_.notify_one();

  Stg::World::Run(); // Note: This must be run in the same thread as new Stg::WorldGui(..) or FLTK will crash with a GL error
  // Todo: tell someone that we have stopped
}

StageComponent::StageComponent(const std::string& instance_name, darc::Node::Ptr node) :
  darc::Component(instance_name, node),
  init_done_(false)
{
  ros::Time::init(); // Need to call this to make ros::Time::now() work

  boost::thread thread = boost::thread(boost::bind(&StageComponent::runStageThread, this ));

  boost::unique_lock<boost::mutex> lock(init_mutex_);
  while(!init_done_)
  {
    init_condition_.wait(lock);
  }

}

DARC_REGISTER_COMPONENT(StageComponent);
