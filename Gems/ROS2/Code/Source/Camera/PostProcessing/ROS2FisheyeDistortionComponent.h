/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "Atom/Feature/ParamMacros/MapAllCommon.inl"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>
#include <Atom/Feature/CubeMapCapture/CubeMapCaptureFeatureProcessorInterface.h>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>

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
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2FisheyeDistortionComponent, "{B47D9F3A-C812-4E5F-A0D7-823E1CF94B56}");
        static void Reflect(AZ::ReflectContext* context);

        ROS2FisheyeDistortionComponent() = default;
        ~ROS2FisheyeDistortionComponent() override = default;

        void Activate() override;
        void Deactivate() override;


        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        AZ::Render::CubeMapCaptureHandle  m_cubeMapCaptureHandle;
        AZ::Render::CubeMapCaptureFeatureProcessorInterface* m_cubeMapCaptureFeatureProcessor = nullptr;
    private:
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_imagePublisher;
        bool m_inprogress = false;
    };
} // namespace ROS2