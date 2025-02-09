#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

#include <geometry_msgs/msg/point.hpp>
#include <gz/sim/components/Name.hh>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <unordered_map>
#include <vector>

using namespace gz;
using namespace sim;
using namespace systems;
using namespace math;

class PositionSystem : public System, public ISystemConfigure, public ISystemPostUpdate
{
  public:
    virtual void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm, EventManager &_eventMgr) override
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
        node = rclcpp::Node::make_shared("position_system");
        world = World(_entity);

        std::vector<std::string> modelNames = {
            "green_supply_zone", "blue_supply_zone", "drop_zone_1",     "drop_zone_2",     "drop_zone_3",
            "drop_zone_4",       "x500_safmc_d2_1",  "x500_safmc_d2_2", "x500_safmc_d2_3", "x500_safmc_d2_4"};

        for (const auto &modelName : modelNames)
        {
            positionPublishers[modelName] = node->create_publisher<geometry_msgs::msg::Point>("/pose/" + modelName, 1);
        }
    }

    virtual void PostUpdate(const UpdateInfo &_info, const EntityComponentManager &_ecm) override
    {
        for (auto &[modelName, publisher] : positionPublishers)
        {
            Entity entity = world.ModelByName(_ecm, modelName);
            if (entity != kNullEntity)
            {
                PublishPosition(entity, _ecm, publisher);
            }
            else
            {
                gzwarn << "Failed to find model [" + modelName + "]" << std::endl;
            }
        }
    }

  private:
    World world;
    rclcpp::Node::SharedPtr node;
    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr> positionPublishers;

    void PublishPosition(const Entity &entity, const EntityComponentManager &_ecm,
                         const rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr &publisher)
    {
        auto pose = worldPose(entity, _ecm);
        geometry_msgs::msg::Point msg;
        msg.x = pose.Pos().X();
        msg.y = pose.Pos().Y();
        msg.z = pose.Pos().Z();
        publisher->publish(msg);
    }
};

GZ_ADD_PLUGIN(PositionSystem, gz::sim::System, PositionSystem::ISystemConfigure, PositionSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(PositionSystem, "gz::sim::systems::PositionSystem")
