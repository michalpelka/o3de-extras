/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FisheyeDistortionComponent.h"
#include <AzCore/Math/MathUtils.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>

#include <cmath>
#include <vector>

namespace ROS2
{
    void ROS2FisheyeDistortionComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2FisheyeDistortionComponent, AZ::Component>()
                ->Version(0)
                ->Field("Priority", &ROS2FisheyeDistortionComponent::m_priority)
                ->Field("InputVerticalFovDeg", &ROS2FisheyeDistortionComponent::m_inputVerticalFovDeg)
                ->Field("FisheyeFocalLength", &ROS2FisheyeDistortionComponent::m_fisheyeFocalLength);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2FisheyeDistortionComponent>(
                      "Fisheye Distortion Component",
                      "Applies fisheye equidistant distortion (r = f * theta) to a pinhole camera image")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FisheyeDistortionComponent::m_priority,
                        "Priority",
                        "Priority of the post processing. Higher value means later execution.")
                    ->Attribute(AZ::Edit::Attributes::Min, CameraPostProcessingRequests::MIN_PRIORITY)
                    ->Attribute(AZ::Edit::Attributes::Max, CameraPostProcessingRequests::MAX_PRIORITY)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FisheyeDistortionComponent::m_inputVerticalFovDeg,
                        "Input Vertical FoV (deg)",
                        "Vertical field of view of the input pinhole image in degrees. "
                        "Must match the camera sensor's configured vertical FoV.")
                    ->Attribute(AZ::Edit::Attributes::Min, 1.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 179.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FisheyeDistortionComponent::m_fisheyeFocalLength,
                        "Fisheye Focal Length (px)",
                        "Focal length of the fisheye output in pixels (equidistant model: r = f * theta). "
                        "For a 480 px tall image, ~152 px yields ~180 degree FoV.")
                    ->Attribute(AZ::Edit::Attributes::Min, 1.0f);
            }
        }
    }

    void ROS2FisheyeDistortionComponent::Activate()
    {
        CameraPostProcessingRequestBus::Handler::BusConnect(GetEntityId());
    }

    void ROS2FisheyeDistortionComponent::Deactivate()
    {
        CameraPostProcessingRequestBus::Handler::BusDisconnect();
    }

    void ROS2FisheyeDistortionComponent::ApplyPostProcessing(sensor_msgs::msg::Image& image)
    {
        // Determine bytes per pixel; only rgb8 and rgba8 are supported.
        int bytesPerPixel = 0;
        if (image.encoding == "rgb8")
        {
            bytesPerPixel = 3;
        }
        else if (image.encoding == "rgba8")
        {
            bytesPerPixel = 4;
        }
        else
        {
            return;
        }

        const int width = static_cast<int>(image.width);
        const int height = static_cast<int>(image.height);

        if (width <= 0 || height <= 0)
        {
            return;
        }

        // Derive pinhole focal length from image height and vertical FoV.
        // Assumes square pixels and principal point at image centre.
        const float vFovRad = AZ::DegToRad(m_inputVerticalFovDeg);
        const float pinholeFocalLength = (height * 0.5f) / std::tan(vFovRad * 0.5f);

        const float cx = width * 0.5f;
        const float cy = height * 0.5f;
        const float f_fisheye = m_fisheyeFocalLength;

        // Allocate output buffer, initialised to black.
        std::vector<uint8_t> output(image.data.size(), 0);

        for (int v = 0; v < height; ++v)
        {
            for (int u = 0; u < width; ++u)
            {
                // Offset of output pixel relative to the fisheye principal point.
                const float dx = static_cast<float>(u) - cx;
                const float dy = static_cast<float>(v) - cy;
                const float r = std::sqrt(dx * dx + dy * dy);

                // Equidistant model: r = f_fisheye * theta  →  theta = r / f_fisheye
                const float theta = r / f_fisheye;

                // Pinhole cameras can only represent angles strictly less than 90°.
                // Skip pixels whose ray lies behind or exactly on the image plane.
                if (theta >= AZ::Constants::HalfPi)
                {
                    continue;
                }

                // Project the 3D ray (described by theta and azimuth phi) onto the
                // source pinhole image plane.
                //   tan(theta) = radial_distance_on_plane / depth
                //   u_src = f_pinhole * tan(theta) * cos(phi) + cx
                //   v_src = f_pinhole * tan(theta) * sin(phi) + cy
                // When r == 0 the ray is along the optical axis; avoid division by zero.
                float u_src = cx;
                float v_src = cy;
                if (r > 1e-6f)
                {
                    const float tanTheta = std::tan(theta);
                    const float scale = pinholeFocalLength * tanTheta / r;
                    u_src = scale * dx + cx;
                    v_src = scale * dy + cy;
                }

                // Bilinear interpolation: clamp to valid integer sample positions.
                const int u0 = static_cast<int>(u_src);
                const int v0 = static_cast<int>(v_src);

                if (u0 < 0 || u0 >= width - 1 || v0 < 0 || v0 >= height - 1)
                {
                    continue;
                }

                const float du = u_src - static_cast<float>(u0);
                const float dv = v_src - static_cast<float>(v0);

                const int dstOffset = (v * width + u) * bytesPerPixel;

                for (int c = 0; c < bytesPerPixel; ++c)
                {
                    const float p00 = static_cast<float>(image.data[(v0 * width + u0) * bytesPerPixel + c]);
                    const float p10 = static_cast<float>(image.data[(v0 * width + (u0 + 1)) * bytesPerPixel + c]);
                    const float p01 = static_cast<float>(image.data[((v0 + 1) * width + u0) * bytesPerPixel + c]);
                    const float p11 = static_cast<float>(image.data[((v0 + 1) * width + (u0 + 1)) * bytesPerPixel + c]);

                    const float interpolated =
                        p00 * (1.0f - du) * (1.0f - dv) +
                        p10 * du * (1.0f - dv) +
                        p01 * (1.0f - du) * dv +
                        p11 * du * dv;

                    output[dstOffset + c] = static_cast<uint8_t>(interpolated + 0.5f);
                }
            }
        }

        // Write the distorted data back into the image message.
        // The encoding, dimensions, and step remain unchanged.
        std::copy(output.begin(), output.end(), image.data.begin());
    }

    AZ::u8 ROS2FisheyeDistortionComponent::GetPriority() const
    {
        return m_priority;
    }

} // namespace ROS2