/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include "AzCore/Math/Aabb.h"
#include "AzCore/std/containers/unordered_map.h"
#include "AzCore/std/containers/unordered_set.h"
#include "AzFramework/Physics/Common/PhysicsTypes.h"
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TickBus.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/RigidBodyBus.h>

namespace ROS2
{

   class ConveyorBeltComponent
        : public AZ::Component
        , public AZ::TickBus::Handler
        , protected Physics::RigidBodyNotificationBus::Handler
   {
   public:
       AZ_COMPONENT(ConveyorBeltComponent, "{61ECBDD3-94B4-4982-96CA-284FA27A8378}");
       ConveyorBeltComponent() = default;
       ~ConveyorBeltComponent() = default;
       static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
       static void Reflect(AZ::ReflectContext* context);
       //////////////////////////////////////////////////////////////////////////
       // Component overrides
       void Activate() override;
       void Deactivate() override;
       //////////////////////////////////////////////////////////////////////////

   private:

       // Physics::RigidBodyNotifications overrides...
       void OnPhysicsEnabled(const AZ::EntityId& entityId) override;
       void OnPhysicsDisabled(const AZ::EntityId& entityId) override;

       void OnTriggerEnter(const AzPhysics::TriggerEvent& triggerEvent);
       void OnTriggerExit(const AzPhysics::TriggerEvent& triggerEvent);

       // AZ::TickBus::Handler overrides
       void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

       void PostPhysicsSubTick(float fixedDeltaTime);

       float m_speed = 1.0f;
       AZ::EntityId m_TempConveyorEntityId;


       AZStd::unordered_map<AZ::EntityId, int> m_entitiesOnBelt;

       AzPhysics::SimulatedBodyEvents::OnTriggerEnter::Handler m_onTriggerEnterHandler;
       AzPhysics::SimulatedBodyEvents::OnTriggerExit::Handler m_onTriggerExitHandler;
       AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneFinishSimHandler;

   };
} // namespace ROS2
