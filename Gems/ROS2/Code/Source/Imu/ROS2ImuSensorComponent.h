/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace ROS2
{

    //! An IMU (Inertial Measurement Unit) sensor Component.
    //! IMUs typically include gyroscopes, accelerometers and magnetometers. This component encapsulates data
    //! acquisition and its publishing to ROS2 ecosystem. IMU Component requires ROS2FrameComponent.
    class ROS2ImuSensorComponent : public ROS2SensorComponent
    {
    public:
        AZ_COMPONENT(ROS2ImuSensorComponent, "{502A955E-7742-4E23-AD77-5E4063739DCA}", ROS2SensorComponent);
        ROS2ImuSensorComponent();
        ~ROS2ImuSensorComponent() = default;
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //! Length of filter that removes numerical noise
        int m_filterSize{ 10 };

        //! Include gravity acceleration
        bool m_includeGravity {true};

        //! Measure also absolute rotation
        bool m_absoluteRotation {true};

        std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> m_imuPublisher;
        sensor_msgs::msg::Imu m_imuMsg;
        AZ::Vector3 m_previousLinearVelocity = AZ::Vector3::CreateZero();

        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_onSceneSimulationEvent;
        AzPhysics::SimulatedBodyHandle m_bodyHandle = AzPhysics::InvalidSimulatedBodyHandle;
        AZ::Vector3 m_acceleration{ 0 };
        AZStd::deque<AZ::Vector3> m_filterAcceleration;
        AZStd::deque<AZ::Vector3> m_filterAngularVelocity;
    protected:
        // ROS2SensorComponent overrides ...
        void SetupRefreshLoop() override;
    };
} // namespace ROS2
