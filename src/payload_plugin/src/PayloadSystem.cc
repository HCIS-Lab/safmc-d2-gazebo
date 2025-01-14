#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <gz/sim/components/Name.hh>

#include "agent_msgs/msg/magnet_control.hpp" 

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
            droneMagnetStatus.resize(numDrones, false);  
            dronePayloadAttachedStatus.resize(numDrones, false); 
            subscriptions.resize(numDrones, nullptr); 
            publishers.resize(numDrones, nullptr);

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
                    std::string topicName = "/drone_" + std::to_string(i) + "/magnet_control";
                    auto callback = [this, i](const agent_msgs::msg::MagnetControl::SharedPtr msg) {
                        this->droneMagnetStatus[i] = msg->magnet1; 
                        if(!msg->magnet1) {
                            dronePayloadAttachedStatus[i] = false;
                            std_msgs::msg::Bool msg;
                            msg.data = dronePayloadAttachedStatus[i];
                            publishers[i]->publish(msg);
                        }
                        RCLCPP_INFO(node_->get_logger(), "Drone %d magnet status updated: %s", i, msg->magnet1 ? "true" : "false");
                    };

                    subscriptions[i] = node_->create_subscription<agent_msgs::msg::MagnetControl>(topicName, rclcpp::QoS(1).best_effort(), callback);

                    publishers[i] = node_->create_publisher<std_msgs::msg::Bool>("/drone_" + std::to_string(i) + "/is_loaded", 1);
                    std_msgs::msg::Bool msg;
                    msg.data = dronePayloadAttachedStatus[i];
                    publishers[i]->publish(msg);
                }
            }
            first = 0;
        }
        
        //update payload pose if within 20 cm
        for (int i = 0; i < numPayload; i++) {
            if (payloadModelEntities[i] == kNullEntity)continue;
            auto payloadPose = worldPose(payloadModelEntities[i], _ecm); 
            auto payloadModel = Model(payloadModelEntities[i]);
            bool isAttached = false;
            for (int j = 0; j < numDrones; j++) {
                if (droneModelEntities[j] == kNullEntity  || !droneMagnetStatus[j] || dronePayloadAttachedStatus[j])continue;
                auto dronePose = worldPose(droneModelEntities[j], _ecm); 
                double dx = dronePose.Pos().X() - payloadPose.Pos().X();
                double dy = dronePose.Pos().Y() - payloadPose.Pos().Y();
                double dz = dronePose.Pos().Z() - payloadPose.Pos().Z();
                if (sqrt(dx * dx + dy * dy + dz * dz) < threshold) { 
                    isAttached = true;
                    Pose3d newPose(dronePose);
                    payloadModel.SetWorldPoseCmd(_ecm, newPose);
                    dronePayloadAttachedStatus[j] = true;
                    std_msgs::msg::Bool msg;
                    msg.data = dronePayloadAttachedStatus[j];
                    publishers[j]->publish(msg);
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
    std::vector<bool> droneMagnetStatus;
    std::vector<bool> dronePayloadAttachedStatus;
    std::vector<rclcpp::Subscription<agent_msgs::msg::MagnetControl>::SharedPtr> subscriptions;
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> publishers;
    rclcpp::Node::SharedPtr node_;
};

GZ_ADD_PLUGIN(PayloadSystem, gz::sim::System, PayloadSystem::ISystemConfigure, PayloadSystem::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(PayloadSystem, "gz::sim::systems::PayloadSystem")
