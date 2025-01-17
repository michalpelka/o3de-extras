/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2Transform.h>
#include <ROS2/Sensor/SensorConfiguration.h>

#include <AzToolsFramework/API/ComponentEntitySelectionBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include "CameraSensor.h"

namespace ROS2
{
    //! ROS2 Camera Editor sensor component class
    //! Allows turning an entity into a camera sensor in Editor
    //! Component draws camera frustrum in the Editor
    class ROS2CameraSensorEditorComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , protected AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        ROS2CameraSensorEditorComponent();
        ~ROS2CameraSensorEditorComponent() override = default;
        AZ_EDITOR_COMPONENT(ROS2CameraSensorEditorComponent, "{3C2A86B2-AD58-4BF1-A5EF-71E0F94A2B42}");
        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        static void GetPr(AZ::ComponentDescriptor::DependencyArrayType& required);
        void Activate() override;
        void Deactivate() override;

        // AzToolsFramework::Components::EditorComponentBase override
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // EntityDebugDisplayEventBus::Handler overrides
        void DisplayEntityViewport(const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        AZStd::pair<AZStd::string, TopicConfiguration> MakeTopicConfigurationPair(
            const AZStd::string& topic, const AZStd::string& messageType, const AZStd::string& configName) const;
        SensorConfiguration m_sensorConfiguration;
        float m_VerticalFieldOfViewDeg = 90.0f;
        int m_width = 640;
        int m_height = 480;
        bool m_colorCamera = true;
        bool m_depthCamera = true;
    };
} // namespace ROS2