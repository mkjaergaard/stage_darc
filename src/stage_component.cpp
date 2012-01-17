// libstage
#include <stage.hh>

#include <boost/filesystem.hpp>
#include <darc/component.h>
#include <stage_darc/stage_robot.h>


// Our node
class StageComponent : public darc::Component
{
protected:
  std::vector<stage_darc::StageRobot*> robots_;

public:
  StageComponent(const std::string& instance_name, darc::Node::Ptr node);

  ~StageComponent() {};

  void RunStageThread();

  int SubscribeModels();

  void update()
  {
    robots_[0]->update();
  }

  // Do one update of the world.  May pause if the next update time
  // has not yet arrived.
  bool UpdateWorld();

  // The main simulator object
  Stg::World* world_;
};

bool s_update(Stg::World* world, StageComponent * comp)
{
  std::cout << "callback" << std::endl;
  //node->WorldCallback();
  // We return false to indicate that we want to be called again (an
  // odd convention, but that's the way that Stage works).
  comp->update();
  return false;
}

void StageComponent::RunStageThread()
{
  const std::string file = "/u/mkjargaard/repositories/mkjargaard/darcbuild/stage_darc/world/simple.world";

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
  world_->AddUpdateCallback((Stg::world_callback_t)s_update, this);

  Stg::World::Run();
  // Todo: tell someone that we have stopped
}

StageComponent::StageComponent(const std::string& instance_name, darc::Node::Ptr node) :
  darc::Component(instance_name, node)
{
  boost::thread thread = boost::thread(boost::bind(&StageComponent::RunStageThread, this ));

  sleep(2);

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

}

DARC_REGISTER_COMPONENT(StageComponent);
