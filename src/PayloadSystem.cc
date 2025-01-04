#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace math;

class PayloadSystem : public System, public ISystemConfigure, public ISystemPreUpdate, public ISystemPostUpdate
{
public:
    virtual void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm, EventManager &_eventMgr) override
    {
        if (!rclcpp::ok())
        {
            rclcpp::init(0, nullptr);
        }

        node_ = rclcpp::Node::make_shared("custom_system");
        subscription_ = node_->create_subscription<std_msgs::msg::Bool>("custom_system", 1, std::bind(&PayloadSystem::topic_callback, this, std::placeholders::_1));

        // --------------------

        auto world = World(_entity);
        std::cout << world.ModelCount(_ecm) << std::endl;

        for (int i = 1; i <= 15; ++i)
        {
            std::string modelName = "payload" + std::to_string(i);
            payloadModelEntities.push_back(world.ModelByName(_ecm, modelName));
        }
        for (const auto &payloadModelEntity : payloadModelEntities)
        {
            if (payloadModelEntity == kNullEntity)
            {
                std::cout << "NULL" << std::endl;
            }
            else
            {
                std::cout << worldPose(payloadModelEntity, _ecm) << std::endl;
            }
        }

        // =------
        this->status = 0;
    }

    virtual void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
    {
        rclcpp::spin_some(this->node_);

        auto model = Model(payloadModelEntities[0]);
        auto pose = worldPose(payloadModelEntities[0], _ecm);

        Pose3d newPose(pose.Pos().X() + 1.0, pose.Pos().Y(), pose.Pos().Z(), pose.Rot().W(), pose.Rot().X(),
                       pose.Rot().Y(), pose.Rot().Z());
        model.SetWorldPoseCmd(_ecm, newPose);

        if (this->status == 0)
        {
            std::cout << "--" << std::endl;
        }
        else if (this->status == 1)
        {
            std::cout << "ON" << std::endl;
        }
        else if (this->status == 2)
        {
            std::cout << "OFF" << std::endl;
        }
        else
        {
            std::cout << "ERROR" << std::endl;
        }
    }

    virtual void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override
    {
    }

private:
    std::vector<Entity> payloadModelEntities;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    rclcpp::Node::SharedPtr node_;
    int status = 0;
    void topic_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            this->status = 1;
        }
        else
        {
            this->status = 2;
        }
    }
};

GZ_ADD_PLUGIN(PayloadSystem, gz::sim::System, PayloadSystem::ISystemConfigure, PayloadSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(PayloadSystem, "gz::sim::systems::PayloadSystem")
