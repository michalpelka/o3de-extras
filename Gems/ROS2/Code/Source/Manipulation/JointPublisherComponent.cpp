#include <ROS2/Manipulation/JointPublisherComponent.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <rclcpp/qos.hpp>
#include <PhysX/Joint/PhysXJointRequestsBus.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void JointPublisherComponent::Activate()
    {
        m_useJoints = false;
        m_useArticulation  = false;
        AZ::TickBus::Handler::BusConnect();
        auto ros2Node = ROS2::ROS2Interface::Get()->GetNode();
        auto ros2Frame = GetEntity()->FindComponent<ROS2FrameComponent>();
        AZStd::string namespacedTopic = ROS2Names::GetNamespacedName(ros2Frame->GetNamespace(), "joint_states");
        m_jointstatePublisher = ros2Node->create_publisher<sensor_msgs::msg::JointState>(namespacedTopic.data(), rclcpp::SystemDefaultsQoS());        // TODO: add QoS instead of "1"
    }

    void JointPublisherComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        m_jointstatePublisher.reset();
    }

    void JointPublisherComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointPublisherService"));
    }

    void JointPublisherComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void JointPublisherComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointPublisherComponent, AZ::Component>()
                ->Version(0)
                ->Field("Frequency (HZ)", &JointPublisherComponent::m_frequency);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointPublisherComponent>("JointPublisherComponent", "[Publish all the Hinge joint in the tree]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &JointPublisherComponent::m_frequency, "Frequency", "Frequency of publishing [Hz]")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                    ->Attribute(AZ::Edit::Attributes::Max, 1000.0f);
            }
        }
    }

    void JointPublisherComponent::Initialize()
    {
        AZStd::vector<AZ::EntityId> descendants;
        AZ::TransformBus::EventResult(descendants, GetEntityId(), &AZ::TransformInterface::GetAllDescendants);

        for (const AZ::EntityId& descendantID : descendants)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, descendantID);
            AZ_Assert(entity, "Unknown entity %s", descendantID.ToString().c_str());
            auto* frameComponent = entity->FindComponent<ROS2FrameComponent>();
            auto* hingeComponent = entity->FindComponent<PhysX::HingeJointComponent>();
            auto* articulationComponent = entity->FindComponent<PhysX::ArticulationLinkComponent>();

            if (frameComponent && hingeComponent)
            {
                AZ::Name jointName = frameComponent->GetJointName();
                AZ_Printf("JointPublisherComponent", "Adding entity %s %s to the hierarchy map with joint name %s\n", entity->GetName().c_str(), descendantID.ToString().c_str(), jointName.GetCStr() );
                m_hierarchyMap[jointName] = AZ::EntityComponentIdPair(entity->GetId(), hingeComponent->GetId());
                m_jointstateMsg.name.push_back(jointName.GetCStr());
                m_jointstateMsg.position.push_back(0.0f);
                m_useJoints = true;
                AZ_Assert(!m_useArticulation, "JointPublisherComponent: Cannot have both joints and articulations in the same tree");
            }
            if (frameComponent && articulationComponent)
            {
                // get free articulation's axis
                bool isArticulationFixed = true;
                for (AZ::u8 axis = 0; axis <= static_cast<AZ::u8>(PhysX::ArticulationJointAxis::Z); axis++)
                {
                    PhysX::ArticulationJointMotionType type =
                        articulationComponent->GetMotion(static_cast<PhysX::ArticulationJointAxis>(axis));
                    if (type != PhysX::ArticulationJointMotionType::Locked)
                    {
                        isArticulationFixed = false;
                        break;
                    }
                }
                if (!isArticulationFixed)
                {
                    AZ::Name jointName = frameComponent->GetJointName();
                    AZ_Printf("JointPublisherComponent", "Adding entity %s %s to the hierarchy map with joint name %s\n", entity->GetName().c_str(), descendantID.ToString().c_str(), jointName.GetCStr() );
                    m_hierarchyMap[jointName] = AZ::EntityComponentIdPair(entity->GetId(), articulationComponent->GetId());
                    m_jointstateMsg.name.push_back(jointName.GetCStr());
                    m_jointstateMsg.position.push_back(0.0f);
                    m_useArticulation = true;
                    AZ_Assert(!m_useJoints, "JointPublisherComponent: Cannot have both joints and articulations in the same tree");
                }
                else
                {
                    AZ_Printf("JointPublisherComponent", "Articulation joint from entity %s to entity %s is fixed, skipping\n", entity->GetName().c_str(), descendantID.ToString().c_str());
                }

            }
        }
    }

    AZStd::unordered_map<AZ::Name, AZ::EntityComponentIdPair>  &JointPublisherComponent::GetHierarchyMap()
    {
        return m_hierarchyMap;
    }

    void JointPublisherComponent::UpdateMessage()
    {
        int i = 0;
        std_msgs::msg::Header ros_header;
        ros_header.frame_id = GetFrameID().data();
        ros_header.stamp = ROS2::ROS2Interface::Get()->GetROSTimestamp();
        m_jointstateMsg.header = ros_header;
        m_jointstateMsg.position.resize(m_hierarchyMap.size());
        m_jointstateMsg.name.resize(m_hierarchyMap.size());
        for (auto& [name, hingeComponent] : m_hierarchyMap)
        {
            m_jointstateMsg.position[i] = GetJointPosition(hingeComponent);
            m_jointstateMsg.name[i] = name.GetCStr();
            i++;
        }
    }

    float JointPublisherComponent::GetJointPosition(const AZ::EntityComponentIdPair idPair) const
    {
        float position{ 0 };
        if (m_useArticulation)
        {
            PhysX::ArticulationJointRequestBus::EventResult(position, idPair.GetEntityId(), &PhysX::ArticulationJointRequests::GetJointPosition,
                                                            PhysX::ArticulationJointAxis::Twist);
        }
        if (m_useJoints)
        {
            PhysX::JointRequestBus::EventResult(position, idPair, &PhysX::JointRequests::GetPosition);
            return position;
        }
        AZ_Assert(false, "JointPublisherComponent: No joints or articulations found in the tree");
        return position;
    }

    void JointPublisherComponent::PublishMessage()
    {
        UpdateMessage();
        m_jointstatePublisher->publish(m_jointstateMsg);
    }

    AZStd::string JointPublisherComponent::GetFrameID() const
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetFrameID();
    }

    void JointPublisherComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (!m_initialized)
        {
            Initialize();
            m_initialized = true;
        }

        AZ_Assert(m_frequency > 0, "JointPublisher frequency must be greater than zero");
        auto frameTime = 1 / m_frequency;

        m_timeElapsedSinceLastTick += deltaTime;
        if (m_timeElapsedSinceLastTick < frameTime)
            return;

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }

        // Note that the publisher frequency can be limited by simulation tick rate (if higher frequency is desired).
        PublishMessage();
    }
} // namespace ROS2
