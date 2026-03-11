/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FisheyeDistortionComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Atom/RPI.Public/Pass/Specific/EnvironmentCubeMapPass.h>
#include <Atom/RPI.Public/Scene.h>

#include <cstring>
#include <AzCore/Component/TransformBus.h>
#include <ROS2/ROS2Bus.h>
namespace ROS2
{
    namespace
    {
        // IEEE 754 half-float → float conversion (handles normals, zeros, inf, nan).
        float HalfToFloat(uint16_t h)
        {
            const uint32_t sign     = static_cast<uint32_t>(h >> 15) << 31;
            const uint32_t exp      = (h >> 10) & 0x1F;
            const uint32_t mantissa = h & 0x3FF;

            uint32_t bits;
            if (exp == 0)
            {
                bits = sign | (mantissa << 13); // zero / denormal → treat as zero
            }
            else if (exp == 31)
            {
                bits = sign | 0x7F800000u | (mantissa << 13); // inf / nan
            }
            else
            {
                bits = sign | ((exp + 112u) << 23) | (mantissa << 13); // normal
            }

            float f;
            std::memcpy(&f, &bits, sizeof(f));
            return f;
        }

        // Simple Reinhard tone-map + gamma 2.2, returns value in [0, 255].
        uint8_t TonemapChannel(float linear)
        {
            linear = linear / (1.0f + linear); // Reinhard
            linear = std::pow(linear < 0.0f ? 0.0f : linear, 1.0f / 2.2f); // gamma
            return static_cast<uint8_t>(linear * 255.0f + 0.5f);
        }

        sensor_msgs::msg::Image BuildCubeMapImageMessage(uint8_t* const* faces)
        {
            constexpr uint32_t faceSize   = AZ::RPI::EnvironmentCubeMapPass::CubeMapFaceSize; // 1024
            constexpr uint32_t numFaces   = 6;
            constexpr uint32_t stripWidth = faceSize * numFaces;

            sensor_msgs::msg::Image msg;
            msg.encoding = "rgb8";
            msg.width    = stripWidth;
            msg.height   = faceSize;
            msg.step     = stripWidth * 3;
            msg.data.resize(msg.step * msg.height);

            for (uint32_t row = 0; row < faceSize; ++row)
            {
                for (uint32_t face = 0; face < numFaces; ++face)
                {
                    const uint16_t* src = reinterpret_cast<const uint16_t*>(faces[face]);
                    src += row * faceSize * 4; // 4 channels per pixel

                    for (uint32_t col = 0; col < faceSize; ++col)
                    {
                        const uint32_t dstOffset = (row * stripWidth + face * faceSize + col) * 3;
                        msg.data[dstOffset + 0] = TonemapChannel(HalfToFloat(src[col * 4 + 0]));
                        msg.data[dstOffset + 1] = TonemapChannel(HalfToFloat(src[col * 4 + 1]));
                        msg.data[dstOffset + 2] = TonemapChannel(HalfToFloat(src[col * 4 + 2]));
                    }
                }
            }

            return msg;
        }
    } // namespace

    void ROS2FisheyeDistortionComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2FisheyeDistortionComponent, AZ::Component>()
                ->Version(0);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2FisheyeDistortionComponent>(
                      "Fisheye Distortion Component",
                      "Captures a cubemap and projects it using the equidistant fisheye model (r = f * theta)")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg");
            }
        }
    }

    void ROS2FisheyeDistortionComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();
        auto node = ROS2::ROS2Interface::Get()->GetNode();
        m_imagePublisher = node->create_publisher<sensor_msgs::msg::Image>("/cubemap", 1);
        m_inprogress = false;
    }

    void ROS2FisheyeDistortionComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        if (m_cubeMapCaptureFeatureProcessor && m_cubeMapCaptureHandle)
        {
            m_cubeMapCaptureFeatureProcessor->RemoveCubeMapCapture(m_cubeMapCaptureHandle);
        }
    }

    void ROS2FisheyeDistortionComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {

        if (!m_cubeMapCaptureFeatureProcessor)
        {
            m_cubeMapCaptureFeatureProcessor =
                AZ::RPI::Scene::GetFeatureProcessorForEntity<AZ::Render::CubeMapCaptureFeatureProcessorInterface>(GetEntityId());
        }
        if (!m_cubeMapCaptureFeatureProcessor)
        {
            AZ_Error("FisheyeDistortion", false, "CubeMapCaptureFeatureProcessor not found on scene");
            return;
        }
        if (!m_cubeMapCaptureHandle)
        {
            const AZ::Transform worldTM = GetEntity()->GetTransform()->GetWorldTM();
            m_cubeMapCaptureHandle = m_cubeMapCaptureFeatureProcessor->AddCubeMapCapture(worldTM);
        }


        if (!m_inprogress)
        {
            m_inprogress = true;
            m_cubeMapCaptureFeatureProcessor->RenderCubeMap(
                m_cubeMapCaptureHandle,
                [this](uint8_t* const* cubeMapTextureData, [[maybe_unused]] const AZ::RHI::Format cubeMapTextureFormat)
                {
                    m_imagePublisher->publish(BuildCubeMapImageMessage(cubeMapTextureData));
                    std::cerr << "Published cubemap image message with timestamp " << ROS2Interface::Get()->GetROSTimestamp().sec << "." << ROS2Interface::Get()->GetROSTimestamp().nanosec << std::endl;
                    m_inprogress = false;
                },

                /*relativePath=*/"");
        }
    }

} // namespace ROS2