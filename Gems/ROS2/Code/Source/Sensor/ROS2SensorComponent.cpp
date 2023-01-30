/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Sensor/ROS2SensorComponent.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void ROS2SensorComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        m_deltaTimeHistogram.Init("onFrequencyTick Delta Time (FPS)", 250, ImGui::LYImGuiUtils::HistogramContainer::ViewType::Histogram, true, 0.0f, 80.0f);
        ImGui::ImGuiUpdateListenerBus::Handler::BusConnect();

    }

    void ROS2SensorComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ImGui::ImGuiUpdateListenerBus::Handler::BusDisconnect();
    }

    void ROS2SensorComponent::Reflect(AZ::ReflectContext* context)
    {
        SensorConfiguration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SensorComponent, AZ::Component>()->Version(1)->Field(
                "SensorConfiguration", &ROS2SensorComponent::m_sensorConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SensorComponent>("ROS2 Sensor", "Base component for sensors")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2SensorComponent::m_sensorConfiguration,
                        "Sensor configuration",
                        "Sensor configuration");
            }
        }
    }

    AZStd::string ROS2SensorComponent::GetNamespace() const
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetNamespace();
    };

    AZStd::string ROS2SensorComponent::GetFrameID() const
    {
        auto* ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
        return ros2Frame->GetFrameID();
    }

    void ROS2SensorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2SensorComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        Visualise(); // each frame
        if (!m_sensorConfiguration.m_publishingEnabled)
        {
            return;
        }

        auto frequency = m_sensorConfiguration.m_frequency;

        auto frameTime = frequency == 0 ? 1 : 1 / frequency;

        m_timeElapsedSinceLastTick += deltaTime;
        if (m_timeElapsedSinceLastTick - frameTime < 0.5f*deltaTime)
            return;

        m_timeElapsedSinceLastTick -= frameTime;
        if (deltaTime > frameTime)
        { // Frequency higher than possible, not catching up, just keep going with each frame.
            m_timeElapsedSinceLastTick = 0.0f;
        }
        // Note that sensor frequency can be limited by simulation tick rate (if higher sensor Hz is desired).
        double currentTime =  TimeMsToSecondsDouble(AZ::Interface<AZ::ITime>::Get()->GetElapsedTimeMs());
        const float deltaTimeOnFreqTick = 1.0f*(currentTime - m_timeElapsedSinceLastFrequencyTick);
        m_effectiveFps  = 1.f / deltaTimeOnFreqTick;
        m_deltaTimeHistogram.PushValue(1.0f/deltaTimeOnFreqTick);
        m_timeElapsedSinceLastFrequencyTick = currentTime;
        m_effectiveFpsMin = AZStd::min(m_effectiveFpsMin, m_effectiveFps);
        m_effectiveFpsMax = AZStd::max(m_effectiveFpsMax, m_effectiveFps);

        FrequencyTick();

    }

    void ROS2SensorComponent::OnImGuiUpdate(){
        AZStd::string s = AZStd::string::format("Sensor %s [%llu] ", GetEntity()->GetName().c_str(), GetId());
        ImGui::Begin(s.c_str());
        ImGui::BulletText("FrameRate : %f / %f", m_effectiveFps, m_sensorConfiguration.m_frequency);
        ImGui::BulletText("Max/Min : %f / %f", m_effectiveFpsMin, m_effectiveFpsMax);
        ImGui::SameLine();
        if (ImGui::Button("Zero")){
            m_effectiveFpsMin = 300.f;
            m_effectiveFpsMax = 0.f;
        }

        m_deltaTimeHistogram.Draw(ImGui::GetColumnWidth(), 100.0f);
        ImGui::End();
    }

} // namespace ROS2
