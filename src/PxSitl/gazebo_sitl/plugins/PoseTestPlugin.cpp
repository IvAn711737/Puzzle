#include "Utils.hpp"
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <map>

using namespace gazebo;
using gazebo::physics::LinkPtr;
using gazebo::physics::ModelPtr;
using ignition::math::Pose3d;
using ignition::math::Quaterniond;
using ignition::math::Vector3d;
using std::cout;
using std::endl;

class GunPlugin : public ModelPlugin {
private:
  double _maxSpeed = 1.;

  transport::NodePtr _node;
  transport::PublisherPtr _factoryPub;
  transport::PublisherPtr _targetPub;
  transport::PublisherPtr _visualPub;
  transport::SubscriberPtr _selectObject;
  transport::SubscriberPtr _shootSub;

  physics::WorldPtr _thisWorld;
  ModelPtr _thisModel;
  ModelPtr _target;
  LinkPtr _baseLink;
  LinkPtr _boxLink;
  LinkPtr _rotLink;

  event::ConnectionPtr _updateWorld;

  common::Time loopTimer;
  common::Time lastFrame;

public:
  GunPlugin() : ModelPlugin(), _node(new transport::Node()) {}
  ~GunPlugin() {}

  void printPose() {
    gzmsg << "---\n";
    gzmsg << "ModelWorld " << _thisModel->WorldPose()
          << " Relative: " << _thisModel->RelativePose() << endl;
    gzmsg << "BaseWorld: " << _baseLink->WorldPose()
          << " Relative: " << _baseLink->RelativePose() << endl;
    gzmsg << "BoxWorld: " << _boxLink->WorldPose()
          << " Relative: " << _boxLink->RelativePose() << endl;
  }

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) {

    _thisWorld = model->GetWorld();
    _thisModel = model;

    _baseLink = model->GetLink("link");
    _boxLink = model->GetLink("link_0");
    _rotLink = model->GetLink("link_1");

    Pose3d originalBasePose = _baseLink->WorldPose();
    Pose3d originalBoxPose = _boxLink->WorldPose();
    Pose3d originalModelPose = _thisModel->WorldPose();

    _node->Init(_thisWorld->Name());
    _factoryPub = _node->Advertise<msgs::Factory>("~/factory", 10, 1);
    _visualPub = _node->Advertise<msgs::Visual>("~/visual");

    _selectObject = _node->Subscribe("~/selection",
                                     &GunPlugin::onSelectObjectCallback, this);

    _updateWorld = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GunPlugin::onWorldUpdate, this, std::placeholders::_1));

    loopTimer = _thisWorld->RealTime();
  }

  void onWorldUpdate(const common::UpdateInfo &worldInfo) {
    common::Time dT = _thisWorld->SimTime() - lastFrame;
    if (worldInfo.realTime >= loopTimer) {

      if (_target) {
        Quaterniond modelRot = _thisModel->WorldPose().Rot();
        Quaterniond targetRot = _target->WorldPose().Rot();
        Vector3d targetPos = _target->WorldPose().Pos();
        Vector3d modelPos = _thisModel->WorldPose().Pos();

        Vector3d targetPosXY(targetPos.X(), targetPos.Y(), 0.f);

        Vector3d rotPos = _rotLink->RelativePose().Pos();
        Vector3d rotXY(rotPos.X(), rotPos.Y(), 0.f);
        
        gazebo::physics::Joint_V js = _rotLink->GetParentJoints();
        Pose3d jointPose = js.at(0)->InitialAnchorPose();

        gzmsg << jointPose.Pos() + rotXY << endl;

        Quaterniond newRot = _rotLink->RelativePose().Rot();
        Quaterniond r = Utils::lookAt(jointPose.Pos() + rotXY, targetPosXY) * dT.Double() * _maxSpeed;

        Pose3d p(jointPose.Pos() + rotXY, newRot + r);

        _rotLink->SetRelativePose(p);
        js.at(0)->
      }

      loopTimer += common::Time(0, common::Time::SecToNano(1 / 30));
    }

    lastFrame = _thisWorld->SimTime();
  }

  void onSelectObjectCallback(ConstSelectionPtr &object) {
    if (object->selected()) {
      _target = _thisWorld->ModelByName(object->name());
      if (_target)
        gzmsg << _target->GetName() << endl;
    }
  }
};

GZ_REGISTER_MODEL_PLUGIN(GunPlugin);