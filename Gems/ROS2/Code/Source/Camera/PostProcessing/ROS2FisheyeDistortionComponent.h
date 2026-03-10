/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>

namespace ROS2
{
    //! Applies fisheye equidistant distortion to a pinhole camera image.
    //!
    //! The equidistant projection model is: r = f * theta
    //! where r is the radial distance in pixels from the principal point,
    //! f is the fisheye focal length in pixels, and theta is the angle
    //! from the optical axis.
    //!
    //! The component performs inverse warping: for each output pixel it computes
    //! the corresponding source direction (via the equidistant model), then
    //! projects that direction onto the input pinhole image using bilinear
    //! interpolation. Pixels that fall outside the input image are set to black.
    //!
    //! The input pinhole focal length is derived from the image height and the
    //! configured vertical field of view, assuming square pixels and the principal
    //! point at the image centre.
    class ROS2FisheyeDistortionComponent
        : public AZ::Component
        , public CameraPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2FisheyeDistortionComponent, "{B47D9F3A-C812-4E5F-A0D7-823E1CF94B56}");
        static void Reflect(AZ::ReflectContext* context);

        ROS2FisheyeDistortionComponent() = default;
        ~ROS2FisheyeDistortionComponent() override = default;

        void Activate() override;
        void Deactivate() override;

        //! CameraPostProcessingRequestBus::Handler overrides
        void ApplyPostProcessing(sensor_msgs::msg::Image& image) override;
        AZ::u8 GetPriority() const override;

    private:
        AZ::u8 m_priority = CameraPostProcessingRequests::DEFAULT_PRIORITY;

        //! Vertical field of view of the input pinhole image in degrees.
        //! Must match the camera sensor's configured vertical FoV.
        float m_inputVerticalFovDeg = 90.0f;

        //! Focal length of the fisheye output in pixels (equidistant model: r = f * theta).
        //! Controls how wide the fisheye view is: a smaller value captures a wider angle.
        //! For a 480-pixel-tall image, ~152 px yields ~180 degree FoV.
        float m_fisheyeFocalLength = 240.0f;
    };
} // namespace ROS2