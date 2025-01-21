#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <gz/sim/components/Name.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "agent_msgs/msg/magnet.hpp"
#include "agent_msgs/msg/payload.hpp"

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
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
        node = rclcpp::Node::make_shared("payload_system");
        world = World(_entity);

        payloadModelEntities.resize(numPayloads, kNullEntity);

        droneModelEntities.resize(numDrones, kNullEntity);
        isMagnetOn.resize(numDrones, false);
        assignedPayload.resize(numDrones, -1);
        MagnetSubscriptions.resize(numDrones, nullptr);
        payloadPublishers.resize(numDrones, nullptr);
    }

    virtual void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override
    {

        rclcpp::spin_some(this->node);

        // update payload pose if within 20 cm
        for (int i = 0; i < numPayloads; ++i)
        {
            if (payloadModelEntities[i] == kNullEntity)
                continue;

            auto payloadModel = Model(payloadModelEntities[i]);
            auto payloadPose = worldPose(payloadModelEntities[i], _ecm);

            for (int j = 0; j < numDrones; j++)
            {
                if (droneModelEntities[j] == kNullEntity)
                    continue;

                int previousPayload = assignedPayload[j];
                bool isPayloadAssigned = false;

                auto dronePose = worldPose(droneModelEntities[j], _ecm);
                double dx = dronePose.Pos().X() - payloadPose.Pos().X();
                double dy = dronePose.Pos().Y() - payloadPose.Pos().Y();
                double dz = dronePose.Pos().Z() - payloadPose.Pos().Z();
                if (isMagnetOn[j] && (dx * dx + dy * dy + dz * dz < magneticRange * magneticRange))
                {
                    Pose3d newPayloadPose(dronePose);
                    payloadModel.SetWorldPoseCmd(_ecm, newPayloadPose);
                    assignedPayload[j] = i;
                    isPayloadAssigned = true;
                }
                else
                {
                    assignedPayload[j] = -1;
                }

                // payload assignment 改變時才 publish
                if (previousPayload != assignedPayload[j])
                {
                    PublishIsLoaded(j);
                }

                if (isPayloadAssigned)
                    break; // 不考慮多台 drone 對同一個 payload 作用的情況
            }
        }
    }

    virtual void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override
    {
        // read payload models (12)
        for (int i = 0; i < numPayloads; ++i)
        {
            std::ostringstream oss;
            oss << "payload_" << std::setw(2) << std::setfill('0') << (i + 1);
            std::string payloadModelName = oss.str();

            payloadModelEntities[i] = world.ModelByName(_ecm, payloadModelName);
        }

        // read drone models (4)
        for (int i = 0; i < numDrones; ++i)
        {
            std::string droneModelName = "x500_safmc_d2_" + std::to_string(i);
            droneModelEntities[i] = world.ModelByName(_ecm, droneModelName);

            if (droneModelEntities[i] == kNullEntity)
                continue;

            if (MagnetSubscriptions[i] == nullptr)
            {
                auto callback = [this, i](const agent_msgs::msg::Magnet::SharedPtr msg) {
                    this->isMagnetOn[i] = msg->magnet1;
                    RCLCPP_INFO(node->get_logger(), "Drone %d magnet status updated: %s", i,
                                this->isMagnetOn[i] ? "ON" : "OFF");
                };

                MagnetSubscriptions[i] = node->create_subscription<agent_msgs::msg::Magnet>(
                    "/drone_" + std::to_string(i) + "/in/magnet", rclcpp::QoS(1).best_effort(), callback);
            }

            if (payloadPublishers[i] == nullptr)
            {
                payloadPublishers[i] =
                    node->create_publisher<agent_msgs::msg::Payload>("/drone_" + std::to_string(i) + "/out/payload", 1);
                PublishIsLoaded(i);
            }
        }
    }

  private:
    World world;
    const double magneticRange = 0.2;
    const int numDrones = 4;
    const int numPayloads = 12;
    rclcpp::Node::SharedPtr node;

    std::vector<Entity> payloadModelEntities;
    std::vector<Entity> droneModelEntities;

    /// @brief 電磁鐵開/關
    std::vector<bool> isMagnetOn;

    /// @brief drone 目前撿起的 payload ID (0 - 11), -1 代表 NOT load
    std::vector<int> assignedPayload;

    /// @brief topic `/drone_?/in/magnet`
    std::vector<rclcpp::Subscription<agent_msgs::msg::Magnet>::SharedPtr> MagnetSubscriptions;

    /// @brief topic `/drone_?/out/payload`
    std::vector<rclcpp::Publisher<agent_msgs::msg::Payload>::SharedPtr> payloadPublishers;

    void PublishIsLoaded(int droneIndex)
    {
        if (payloadPublishers[droneIndex])
        {
            agent_msgs::msg::Payload msg;
            msg.slot1 = (assignedPayload[droneIndex] != -1);
            msg.slot2 = false;
            msg.slot3 = false;
            payloadPublishers[droneIndex]->publish(msg);
        }
    }
};

GZ_ADD_PLUGIN(PayloadSystem, gz::sim::System, PayloadSystem::ISystemConfigure, PayloadSystem::ISystemPreUpdate,
              PayloadSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(PayloadSystem, "gz::sim::systems::PayloadSystem")
