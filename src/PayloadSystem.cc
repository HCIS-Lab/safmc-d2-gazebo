#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gz/sim/components/Name.hh>

#include <iomanip> 
#include <sstream>

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
        if (!rclcpp::ok())rclcpp::init(0, nullptr);
        node_ = rclcpp::Node::make_shared("custom_system");
        world = World(_entity);
        std::cout << world.ModelCount(_ecm) << std::endl;

        //init payload
        for (int i = 0; i < numPayload; ++i)
        {
            std::ostringstream oss;
            oss << "payload_" << std::setw(2) << std::setfill('0') << (i + 1);
            std::string modelName = oss.str();
            auto entity = world.ModelByName(_ecm, modelName);
            payloadModelEntities.push_back(entity);
            if(entity == kNullEntity)
                std::cout << "NULL payload" << std::endl;
            else
                std::cout << modelName << " : " << worldPose(entity, _ecm) << std::endl;
        }
    }

    virtual void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
    {
        rclcpp::spin_some(this->node_);

        if(first){
            //init drones
            droneGrabStatus.resize(numDrones, false);  
            for (int i = 0; i < numDrones; ++i)
            {
                std::string modelName = "x500_safmc_d2_" + std::to_string(i);
                auto entity = world.ModelByName(_ecm, modelName);
                droneModelEntities.push_back(entity);
                if (entity == kNullEntity)
                    std::cout << "NULL drone: " << modelName << std::endl;
                else
                {
                    std::cout << modelName << " : " << worldPose(entity, _ecm) << std::endl;
                    std::string topicName = "/drone_" + std::to_string(i) + "/grab_status";
                    auto callback = [this, i](const std_msgs::msg::Bool::SharedPtr msg) {
                        this->droneGrabStatus[i] = msg->data;
                        RCLCPP_INFO(node_->get_logger(), "Drone %d grab status updated: %s", i, msg->data ? "true" : "false");
                    };

                    auto subscription = node_->create_subscription<std_msgs::msg::Bool>(topicName, rclcpp::QoS(1).best_effort(), callback);
                    subscriptions.push_back(subscription);
                }
            }
            first = 0;
        }
        
        //update payload pose if below the done and within 10 cm
        for (int i = 0; i < numPayload; i++) {
            if (payloadModelEntities[i] == kNullEntity)continue;
            auto payloadPose = worldPose(payloadModelEntities[i], _ecm); 
            auto payloadModel = Model(payloadModelEntities[i]);
            bool isAttached = false;
            for (int j = 0; j < numDrones; j++) {
                if (droneModelEntities[j] == kNullEntity  || !droneGrabStatus[j])continue;
                auto dronePose = worldPose(droneModelEntities[j], _ecm); 
                double dx = dronePose.Pos().X() - payloadPose.Pos().X();
                double dy = dronePose.Pos().Y() - payloadPose.Pos().Y();
                double dz = dronePose.Pos().Z() - payloadPose.Pos().Z();
                if (sqrt(dx * dx + dy * dy + dz * dz) < threshold) { 
                    isAttached = true;
                    Pose3d newPose(dronePose);
                    payloadModel.SetWorldPoseCmd(_ecm, newPose);
                    break;
                }
            }
            if (!isAttached) {
                //should re-apply gravity
            }
        }
    }

    virtual void PostUpdate(const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override
    {
    }

private:
    World world;
    const double threshold = 0.2;
    const int numDrones = 4;
    const int numPayload = 12;
    bool first = 1;
    std::vector<Entity> payloadModelEntities;
    std::vector<Entity> droneModelEntities;
    std::vector<bool> droneGrabStatus;
    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> subscriptions;
    rclcpp::Node::SharedPtr node_;
};

GZ_ADD_PLUGIN(PayloadSystem, gz::sim::System, PayloadSystem::ISystemConfigure, PayloadSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(PayloadSystem, "gz::sim::systems::PayloadSystem")
