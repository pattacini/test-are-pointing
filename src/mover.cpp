// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <functional>
#include <cmath>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/LogStream.h>

namespace gazebo {

/******************************************************************************/
class Mover : public gazebo::ModelPlugin
{
    gazebo::physics::ModelPtr model;
    gazebo::event::ConnectionPtr renderer_connection;
    yarp::os::BufferedPort<yarp::os::Bottle> port;
    ignition::math::Pose3d robot_pose;

    /**************************************************************************/
    void onWorldFrame() {
        if (auto* b = port.read(false)) {
            if (b->size() >= 3) { 
                const auto x = b->get(0).asFloat64();
                const auto y = b->get(1).asFloat64();
                const auto z = b->get(2).asFloat64() + 0.63;
                const auto& rot = model->WorldPose().Rot();
                ignition::math::Pose3d new_pose(x, y, z, rot.W(), rot.X(), rot.Y(), rot.Z());
                model->SetWorldPose(new_pose);
            }
        }
    }

public:
    /**************************************************************************/
    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr) override {
        this->model = model;
        port.open("/" + model->GetName());
        auto bind = std::bind(&Mover::onWorldFrame, this);
        renderer_connection = gazebo::event::Events::ConnectWorldUpdateBegin(bind);

        const auto& world = model->GetWorld();
        const auto& robot = world->ModelByName("iCub");
        robot_pose = robot->WorldPose();    // not working
    }

    virtual ~Mover() {
        if (!port.isClosed()) {
            port.close();
        }
    }
};

}

GZ_REGISTER_MODEL_PLUGIN(gazebo::Mover)
