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
#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Spline.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/deque.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBodyEvents.h>
#include <AzFramework/Physics/RigidBodyBus.h>
namespace ROS2
{

    class ConveyorBeltComponentKinematic
        : public AZ::Component
        , public AZ::TickBus::Handler
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_COMPONENT(ConveyorBeltComponentKinematic, "{B7F56411-01D4-48B0-8874-230C58A578BD}");
        ConveyorBeltComponentKinematic() = default;
        ~ConveyorBeltComponentKinematic() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        // EntityDebugDisplayEventBus
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplayRequests) override;

        float GertSplineLength(AZ::ConstSplinePtr splinePtr);

        //! Obtain the start and end point of the simulated conveyor belt
        //! @param splinePtr the spline to obtain the start and end point from
        //! @return a pair of vectors, the first being the start point and the second being the end point
        AZStd::pair<AZ::Vector3, AZ::Vector3> GetStartAndEndPointOfBelt(AZ::ConstSplinePtr splinePtr);

        //! Obtain the length of the spline
        //! @param splinePtr the spline to obtain the length from
        float GetSplineLength(AZ::ConstSplinePtr splinePtr);

        //! Obtains location of the segment of the belt.
        //! @param sceneHandle the scene handle of the scene the belt is in
        //! @param handle the handle of the simulated body of the segment
        //! @return the location of the segment in world space
        AZ::Vector3 GetLocationOfSegment(AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle handle);

        //! Obtains the transform of of the pose on the spline at the given distance
        //! @param splinePtr the spline to obtain the transform from
        //! @param distanceNormalized the distance along the spline to obtain the transform from (normalized)
        //! @return the transform of the pose on the spline at the given distance
        AZ::Transform GetTransformFromSpline(AZ::ConstSplinePtr splinePtr, float distanceNormalized);

        //! Spawn rigid body at the given location
        //! @param splinePtr the spline to spawn the rigid body on
        //! @param physicsSystem the physics system to spawn the rigid body in
        //! @param sceneHandle the scene handle of the scene to spawn the rigid body in
        //! @param location the location to spawn the rigid body at (normalized)
        //! @return a pair of the normalized location and the handle of the simulated body
        AZStd::pair<float, AzPhysics::SimulatedBodyHandle> CreateSegment(
            AZ::ConstSplinePtr splinePtr,
            AzPhysics::SystemInterface* physicsSystem,
            AzPhysics::SceneHandle sceneHandle,
            float normalizedLocation);

        // AZ::TickBus::Handler overrides
        void OnTick(float delta, AZ::ScriptTimePoint timePoint) override;

        // Every physic update
        void PostPhysicsSubTick(float fixedDeltaTime);

        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_sceneFinishSimHandler;

        float m_speed = 1.0f;
        float m_beltWidth = 1.0f;
        float m_segmentLength = 1.0f;
        AZStd::deque<AZStd::pair<float, AzPhysics::SimulatedBodyHandle>> m_ConveyorSegments;
        static constexpr float m_segmentWidth = 0.1f;

        bool m_initilized{ false };
        float m_splineLength{ -1.f };
        AZ::Vector3 m_startPoint;
        AZ::Vector3 m_endPoint;
        AZ::ConstSplinePtr m_splineConsPtr{ nullptr };
    };
} // namespace ROS2
