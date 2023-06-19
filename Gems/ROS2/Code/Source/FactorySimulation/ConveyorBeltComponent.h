/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TickBus.h>

namespace ROS2
{

   class ConveyorBeltComponent
        : public AZ::Component
        , public AZ::TickBus::Handler

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
       // AZ::TickBus::Handler overrides
       void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

       float m_speed = 1.0f;
       AZ::EntityId m_TempConveyorEntityId;

   };
} // namespace ROS2
