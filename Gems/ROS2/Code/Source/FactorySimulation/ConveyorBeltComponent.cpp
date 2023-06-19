/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ConveyorBeltComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <Source/RigidBodyComponent.h>

namespace ROS2
{

    void ConveyorBeltComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ConveyorBeltComponent>()
                ->Version(1)
                ->Field("Speed", &ConveyorBeltComponent::m_speed)
                ->Field("m_TempConveyorEntityId", &ConveyorBeltComponent::m_TempConveyorEntityId);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ConveyorBeltComponent>("Conveyor Belt Component", "Conveyor Belt Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ConveyorBeltComponent::m_speed,
                        "Speed",
                        "Speed of the conveyor belt")
                    ->DataElement(
                        AZ::Edit::UIHandlers::EntityId,
                        &ConveyorBeltComponent::m_TempConveyorEntityId,
                        "Conveyor Belt Entity (TODO REMOVE)",
                        "Entity of the conveyor belt(TODO REMOVE)");
            }
        }
    }

    void ConveyorBeltComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        //        required.push_back(AZ_CRC_CE("PhysicsRigidBodyService"));
    }

    void ConveyorBeltComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
    }

    void ConveyorBeltComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ConveyorBeltComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {
        AZ_Printf("ConveyorBeltComponent", "ConveyorBeltComponent::OnTick");
    }

} // namespace ROS2
