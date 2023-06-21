/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ConveyorBeltComponentKinematic.h"
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <Source/RigidBodyComponent.h>
namespace ROS2
{
    void ConveyorBeltComponentKinematic::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ConveyorBeltComponentKinematic>()->Version(1)->Field("Speed", &ConveyorBeltComponentKinematic::m_speed);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ConveyorBeltComponentKinematic>("Conveyor Belt Component Kinematic", "Conveyor Belt Component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &ConveyorBeltComponentKinematic::m_speed, "Speed", "Speed of the conveyor belt");
            }
        }
    }

    void ConveyorBeltComponentKinematic::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void ConveyorBeltComponentKinematic::Activate()
    {
        m_initilized = false;
        AZ::TickBus::Handler::BusConnect();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
    }

    void ConveyorBeltComponentKinematic::Deactivate()
    {
        m_sceneFinishSimHandler.Disconnect();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    AZ::Vector3 ConveyorBeltComponentKinematic::GetLocationOfSegment(
        AzPhysics::SceneHandle sceneHandle, AzPhysics::SimulatedBodyHandle handle)
    {
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface");
        auto* body = azdynamic_cast<AzPhysics::RigidBody*>(sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, handle));
        AZ::Vector3 beginOfSegments = body->GetPosition();
        return beginOfSegments;
    }

    AZStd::pair<float, AzPhysics::SimulatedBodyHandle> ConveyorBeltComponentKinematic::CreateSegment(
        AZ::ConstSplinePtr splinePtr,
        AzPhysics::SystemInterface* physicsSystem,
        AzPhysics::SceneHandle sceneHandle,
        float normalizedLocation)
    {
        auto shapeConfiguration =
            AZStd::make_shared<Physics::BoxShapeConfiguration>(AZ::Vector3(m_segmentLength, m_beltWidth, m_segmentWidth));
        const auto transform = GetTransformFromSpline(m_splineConsPtr, normalizedLocation);
        AzPhysics::RigidBodyConfiguration conveyorSegmentRigidBodyConfig;
        conveyorSegmentRigidBodyConfig.m_kinematic = true;
        conveyorSegmentRigidBodyConfig.m_position = transform.GetTranslation();
        conveyorSegmentRigidBodyConfig.m_orientation = transform.GetRotation();
        conveyorSegmentRigidBodyConfig.m_colliderAndShapeData =
            AzPhysics::ShapeColliderPair(AZStd::make_shared<Physics::ColliderConfiguration>(), shapeConfiguration);
        conveyorSegmentRigidBodyConfig.m_computeCenterOfMass = true;
        conveyorSegmentRigidBodyConfig.m_computeInertiaTensor = true;
        conveyorSegmentRigidBodyConfig.m_angularDamping = 0.01f;
        conveyorSegmentRigidBodyConfig.m_linearDamping = 0.01f;
        conveyorSegmentRigidBodyConfig.m_startSimulationEnabled = true;
        conveyorSegmentRigidBodyConfig.m_computeMass = false;
        conveyorSegmentRigidBodyConfig.m_mass = 1.0f;
        AzPhysics::SimulatedBodyHandle handle = physicsSystem->GetScene(sceneHandle)->AddSimulatedBody(&conveyorSegmentRigidBodyConfig);
        return AZStd::make_pair(normalizedLocation, handle);
    }

    void ConveyorBeltComponentKinematic::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system");
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface");
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");

        if (!m_initilized)
        {
            AZ::ConstSplinePtr splinePtr{ nullptr };
            LmbrCentral::SplineComponentRequestBus::EventResult(
                splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
            AZ_Assert(splinePtr, "Spline pointer is null");
            m_splineConsPtr = splinePtr;
            m_sceneFinishSimHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
                [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltatime)
                {
                    this->PostPhysicsSubTick(fixedDeltatime);
                },
                aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
            sceneInterface->RegisterSceneSimulationFinishHandler(defaultSceneHandle, m_sceneFinishSimHandler);
            const auto& [startPoint, endPoint] = GetStartAndEndPointOfBelt(splinePtr);
            m_startPoint = startPoint;
            m_endPoint = endPoint;
            m_splineLength = GetSplineLength(splinePtr);

            // inital segment population
            const float normalizedDistanceStep = 0.5f * m_segmentLength / m_splineLength ;
            for (float f = 0.f; f < 1.f; f += normalizedDistanceStep)
            {
                m_ConveyorSegments.push_back(CreateSegment(splinePtr, physicsSystem, defaultSceneHandle, f));
            }
            AZ_Printf("ConveyorBeltComponentKinematic", "Initial Number of segments: %d", m_ConveyorSegments.size());
            m_initilized = true;
        }

        // spawn first segment if needed
        if (!m_ConveyorSegments.empty())
        {
            AZ::Vector3 position = GetLocationOfSegment(defaultSceneHandle, m_ConveyorSegments.front().second);
            AZ::Vector3 distance = m_startPoint - position;
            if (distance.GetLength() > m_segmentLength / 2.f)
            {
                m_ConveyorSegments.push_front(CreateSegment(m_splineConsPtr, physicsSystem, defaultSceneHandle, 0.f));
            }
        }

        // despawn segments that are at the end of the spline
        for (auto& [pos, handle] : m_ConveyorSegments)
        {
            if (pos > 1.0f)
            {
                sceneInterface->RemoveSimulatedBody(defaultSceneHandle, handle);
                handle = AzPhysics::InvalidSimulatedBodyHandle;
            }
        }

        // clear object handle from cache
        m_ConveyorSegments.erase(
            AZStd::remove_if(
                m_ConveyorSegments.begin(),
                m_ConveyorSegments.end(),
                [](AZStd::pair<float, AzPhysics::SimulatedBodyHandle>& c)
                {
                    return c.second == AzPhysics::InvalidSimulatedBodyHandle;
                }),
            m_ConveyorSegments.end());
    }
    AZStd::pair<AZ::Vector3, AZ::Vector3> ConveyorBeltComponentKinematic::GetStartAndEndPointOfBelt(AZ::ConstSplinePtr splinePtr)
    {
        AZ::Transform splineTransform = AZ::Transform::Identity();
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);
        const AZ::SplineAddress addressBegin = splinePtr->GetAddressByFraction(0.f);
        const AZ::SplineAddress addressEnd = splinePtr->GetAddressByFraction(1.f);
        const AZ::Vector3 posBegin = splinePtr->GetPosition(addressBegin);
        const AZ::Vector3 posEnd = splinePtr->GetPosition(addressEnd);
        return { posBegin, posEnd };
    }

    AZ::Transform ConveyorBeltComponentKinematic::GetTransformFromSpline(AZ::ConstSplinePtr splinePtr, float distanceNormalized)
    {
        AZ_Assert(splinePtr, "Spline pointer is null");
        AZ::SplineAddress address = splinePtr->GetAddressByFraction(distanceNormalized);
        const AZ::Vector3& p = splinePtr->GetPosition(address);
        const AZ::Vector3& v1 = splinePtr->GetTangent(address);
        const AZ::Vector3& v2 = splinePtr->GetNormal(address);
        const AZ::Vector3 v3 = v1.Cross(v2);

        AZ::Matrix3x3 rotationMatrix = AZ::Matrix3x3::CreateFromColumns(v1, v2, v3);
        AZ::Transform transform =
            AZ::Transform::CreateFromMatrix3x3AndTranslation(rotationMatrix, p - AZ::Vector3::CreateAxisZ(m_segmentWidth / 2.f));
        return transform;
    }

    float ConveyorBeltComponentKinematic::GetSplineLength(AZ::ConstSplinePtr splinePtr)
    {
        AZ::Transform splineTransform = AZ::Transform::Identity();
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);
        const AZ::SplineAddress addressEnd = splinePtr->GetAddressByFraction(1.f);
        return splinePtr->GetLength(addressEnd);
    }

    void ConveyorBeltComponentKinematic::DisplayEntityViewport(
        [[maybe_unused]] const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplayRequests)
    {
        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Spline pointer is null");

        AZ::Transform splineTransform = AZ::Transform::Identity();
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);
        for (float f = 0.0; f < 1.0f; f += 0.01)
        {
            AZ::SplineAddress address = splinePtr->GetAddressByFraction(f);
            AZ::Vector3 pos = splinePtr->GetPosition(address);
            pos = splineTransform.TransformPoint(pos);
            debugDisplayRequests.SetColor(AZ::Colors::Red);
            debugDisplayRequests.DrawPoint(pos, 2);
        }
    }

    void ConveyorBeltComponentKinematic::PostPhysicsSubTick(float fixedDeltaTime)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system");

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene intreface");

        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");

        for (auto& [pos, handle] : m_ConveyorSegments)
        {
            auto* body = azdynamic_cast<AzPhysics::RigidBody*>(sceneInterface->GetSimulatedBodyFromHandle(defaultSceneHandle, handle));
            if (body)
            {
                pos += m_speed * fixedDeltaTime / m_splineLength;
                auto transform = GetTransformFromSpline(m_splineConsPtr, pos);
                transform.SetTranslation(transform.GetTranslation());
                body->SetKinematicTarget(transform);
            }
        }
    }

} // namespace ROS2
