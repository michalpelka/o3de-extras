/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ConveyorBeltComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ConveyorBeltComponent::m_speed, "Speed", "Speed of the conveyor belt")
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
        required.push_back(AZ_CRC_CE("SplineService"));
        required.push_back(AZ_CRC_CE("PhysicsColliderService"));
    }

    void ConveyorBeltComponent::Activate()
    {
        m_onTriggerEnterHandler = AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                AZ_Printf("ConveyorBeltComponent", "OnTriggerEnter");
                auto boxId = event.m_otherBody->GetEntityId();
                if (m_entitiesOnBelt.contains(boxId))
                {
                    m_entitiesOnBelt[boxId]++;
                }
                else
                {
                    m_entitiesOnBelt[boxId] = 1;
                }
            });

        m_onTriggerExitHandler = AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler(
            [&]([[maybe_unused]] AzPhysics::SimulatedBodyHandle bodyHandle, [[maybe_unused]] const AzPhysics::TriggerEvent& event)
            {
                AZ_Printf("ConveyorBeltComponent", "OnTriggerExit");
                auto boxId = event.m_otherBody->GetEntityId();
                if (m_entitiesOnBelt.contains(boxId))
                {
                    auto& count = m_entitiesOnBelt[boxId];
                    count--;
                    if (count <= 0)
                    {
                        m_entitiesOnBelt.erase(boxId);
                    }
                }
            });

        Physics::RigidBodyNotificationBus::Handler::BusConnect(GetEntityId());
        AZ::TickBus::Handler::BusConnect();
    }

    void ConveyorBeltComponent::Deactivate()
    {
        m_onTriggerEnterHandler.Disconnect();
        m_onTriggerExitHandler.Disconnect();
        AZ::TickBus::Handler::BusDisconnect();
        Physics::RigidBodyNotificationBus::Handler::BusDisconnect();
    }

    void ConveyorBeltComponent::OnPhysicsEnabled(const AZ::EntityId& entityId)
    {
        AZ_Printf("ConveyorBeltComponent", "OnPhysicsEnabled");

        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system");

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface");

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");

        if (auto* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get())
        {
            AZStd::pair<AzPhysics::SceneHandle, AzPhysics::SimulatedBodyHandle> foundBody =
                physicsSystem->FindAttachedBodyHandleFromEntityId(entityId);
            if (foundBody.first != AzPhysics::InvalidSceneHandle)
            {
                AzPhysics::SimulatedBodyEvents::RegisterOnTriggerEnterHandler(foundBody.first, foundBody.second, m_onTriggerEnterHandler);
                AzPhysics::SimulatedBodyEvents::RegisterOnTriggerExitHandler(foundBody.first, foundBody.second, m_onTriggerExitHandler);
            }
        }

        m_sceneFinishSimHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltatime)
            {
                this->PostPhysicsSubTick(fixedDeltatime);
            },
            aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
        sceneInterface->RegisterSceneSimulationFinishHandler(defaultSceneHandle, m_sceneFinishSimHandler);
    }
    void ConveyorBeltComponent::OnPhysicsDisabled(const AZ::EntityId& entityId)
    {
        m_onTriggerEnterHandler.Disconnect();
        m_onTriggerExitHandler.Disconnect();
    }
    void ConveyorBeltComponent::PostPhysicsSubTick(float fixedDeltaTime)
    {
        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Spline pointer is null");

        AZ::Vector3 conveyeredEntityPositionWorld{ 0 };
        AZ::Quaternion conveyeredEntityRotationWorld{ AZ::Quaternion::CreateIdentity() };

        for (auto& [m_TempConveyorEntityId, _] : m_entitiesOnBelt)
        {
            AZ::TransformBus::EventResult(
                conveyeredEntityPositionWorld, m_TempConveyorEntityId, &AZ::TransformBus::Events::GetWorldTranslation);
            AZ::TransformBus::EventResult(
                conveyeredEntityRotationWorld, m_TempConveyorEntityId, &AZ::TransformBus::Events::GetWorldRotationQuaternion);

            AZ::Transform splineTransform = AZ::Transform::Identity();
            AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);
            splineTransform.Invert();

            const AZ::Vector3 conveyeredEntityPositionLocal = splineTransform.TransformPoint(conveyeredEntityPositionWorld);
            const AZ::Quaternion conveyeredEntityRotationLocal = splineTransform.GetRotation() * conveyeredEntityRotationWorld;

            const AZ::Vector3 boxDirection = conveyeredEntityRotationLocal.TransformVector(AZ::Vector3::CreateAxisX());
            AZ::PositionSplineQueryResult r = splinePtr->GetNearestAddressPosition(conveyeredEntityPositionLocal);

            AZ::Vector3 tangent = splinePtr->GetTangent(r.m_splineAddress);

            Physics::RigidBodyRequestBus::Event(m_TempConveyorEntityId, &Physics::RigidBodyRequests::SetLinearVelocity, tangent * m_speed);

            AZ::Vector3 rot = tangent.Cross(boxDirection);
            Physics::RigidBodyRequestBus::Event(m_TempConveyorEntityId, &Physics::RigidBodyRequests::SetAngularVelocity, -rot * m_speed);
        }
    }
    void ConveyorBeltComponent::OnTick(float deltaTime, AZ::ScriptTimePoint time)
    {

    }

} // namespace ROS2
