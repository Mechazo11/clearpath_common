#!/usr/bin/env python3

"""
Software License Agreement (BSD)

@author    Roni Kreinin <rkreinin@clearpathrobotics.com>
@copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.
Adaptation for Gazebo Harmonic and support for custom YAML file handling by 
@author Azmyin Md. Kamal <azmyin12@gmail.com>.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Clearpath Robotics nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

# Imports
import os
from clearpath_config.common.types.package_path import PackagePath
from clearpath_generator_common.common import BaseGenerator
from clearpath_generator_common.description.attachments import AttachmentsDescription
from clearpath_generator_common.description.links import LinkDescription
from clearpath_generator_common.description.manipulators import ManipulatorDescription
from clearpath_generator_common.description.mounts import MountDescription
from clearpath_generator_common.description.platform import PlatformDescription
from clearpath_generator_common.description.sensors import SensorDescription
from clearpath_generator_common.description.writer import XacroWriter


class DescriptionGenerator(BaseGenerator):
    def __init__(self, setup_path: str = '/etc/clearpath/', robot_yaml: str ='robot.yaml') -> None:
        super().__init__(setup_path, robot_yaml)
        self.xacro_writer = XacroWriter(self.setup_path, self.serial_number)

    def generate(self) -> None:
        # Common Macros
        self.generate_common()
        self.xacro_writer.write_newline()

        # Platform
        self.generate_platform()
        self.xacro_writer.write_newline()

        # Attachments
        self.generate_attachments()
        self.xacro_writer.write_newline()

        # Links
        self.generate_links()
        self.xacro_writer.write_newline()

        # Mounts
        self.generate_mounts()
        self.xacro_writer.write_newline()

        # Sensors
        self.generate_sensors()
        self.xacro_writer.write_newline()

        # Manipulators
        self.generate_manipulators()
        self.xacro_writer.write_newline()

        # Extras
        self.generate_extras()
        self.xacro_writer.write_newline()

        self.xacro_writer.close_file()
        print(f'Generated {self.xacro_writer.file_path}robot.urdf.xacro')

    def generate_common(self) -> None:
        self.xacro_writer.write_comment('Common')
        # Clearpath Common Materials
        self.xacro_writer.write_include(
            package=self.pkg_clearpath_platform_description.get_name(),
            file='common',
            path='urdf/')

    def generate_platform(self) -> None:
        self.platform = self.clearpath_config.platform.get_platform_model()
        platform_description = PlatformDescription(self.platform, self.clearpath_config)

        # Platform macro
        self.xacro_writer.write_comment('Platform')
        self.xacro_writer.write_include(
            package=platform_description.package,
            file=platform_description.file,
            path=platform_description.path)
        self.xacro_writer.write_macro(
            macro=platform_description.macro,
            parameters=platform_description.parameters)

    def generate_attachments(self) -> None:
        self.xacro_writer.write_comment('Attachments')
        self.xacro_writer.write_newline()
        attachments = self.clearpath_config.platform.attachments.get_all()

        for attachment in attachments:
            if attachment.get_enabled():
                attachment_description = AttachmentsDescription(attachment)
                self.xacro_writer.write_include(
                    package=attachment_description.package,
                    file=attachment_description.file,
                    path=attachment_description.path)
                self.xacro_writer.write_macro(
                    macro=attachment_description.file,
                    parameters=attachment_description.parameters,
                    blocks=XacroWriter.add_origin(
                        attachment_description.xyz,
                        attachment_description.rpy))
                self.xacro_writer.write_newline()

    def generate_links(self) -> None:
        self.xacro_writer.write_comment('Links')
        self.xacro_writer.write_newline()
        links = self.clearpath_config.links.get_all_links()

        for link in links:
            link_description = LinkDescription(link)
            self.xacro_writer.write_include(
                package=link_description.package,
                file=link_description.file,
                path=link_description.path)

            self.xacro_writer.write_macro(
                macro=link_description.file,
                parameters=link_description.parameters,
                blocks=XacroWriter.add_origin(
                    link_description.xyz,
                    link_description.rpy))
            self.xacro_writer.write_newline()

    def generate_mounts(self) -> None:
        self.xacro_writer.write_comment('Mounts')
        self.xacro_writer.write_newline()
        mounts = self.clearpath_config.mounts.get_all_mounts()
        for mount in mounts:
            mount_description = MountDescription(mount)

            self.xacro_writer.write_comment(
                '{0}'.format(mount_description.name)
            )

            self.xacro_writer.write_include(
                package=mount_description.package,
                file=mount_description.model,
                path=mount_description.path
            )

            self.xacro_writer.write_macro(
                macro='{0}'.format(mount_description.model),
                parameters=mount_description.parameters,
                blocks=XacroWriter.add_origin(
                    mount_description.xyz, mount_description.rpy)
            )

            self.xacro_writer.write_newline()

    def generate_sensors(self) -> None:
        """
        Generate all sensor xacro.include commands in robot.urdf.xacro file.
        
        self.clearpath_config may have been inherited from BaseGenerator class
        """
        
        self.xacro_writer.write_comment('Sensors')
        self.xacro_writer.write_newline() # Put a new line
        sensors = self.clearpath_config.sensors.get_all_sensors() # List[]
        for sensor in sensors:
            sensor_description = SensorDescription(sensor)
            
            if sensor_description.model == "asus_xtion":
                print(f"A hacky way of adding asus xtion sesnor.")

                # ! WARNING: Bypassing the good stuff in clearpath_config due to time constraints.
                # Only updating what is necessary
                
                # Using BaseDesciption setters
                # sensor_description.parameters: {'name': 'camera_0', 'parent_link': 'fath_pivot_0_mount', 
                #                                'update_rate': 30, 'image_height': 480, 'image_width': 640}
                sensor_description.name = "camera_0"
                sensor_description.package = "asus_xtion_description" # package containing the urdf file
                sensor_description.model = "asus_xtion" # name of the urdf file
                sensor_description.path = "urdf/" # folder containing the urdf file
                
                sensor_description.parameters.update({
                    'update_rate': 30,
                    'image_height': 480,
                    'image_width': 640,
                })

                # "camera_0"
                self.xacro_writer.write_comment(
                    '{0}'.format(sensor_description.name) # str
                )

                # <xacro:include filename="$(find ros2_asus_xtion_description)/urdf/asus_xtion.urdf.xacro"/>
                self.xacro_writer.write_include(
                    package=sensor_description.package, # package
                    file=sensor_description.model, # camera model
                    path=sensor_description.path # path to urdf folder
                )

                # TODO need to ensure xyz and rpy values are correct
                self.xacro_writer.write_macro(
                    macro='{0}'.format(sensor_description.model),
                    parameters=sensor_description.parameters,
                    blocks=XacroWriter.add_origin(
                        sensor_description.xyz, sensor_description.rpy)
                )
            elif sensor_description.model == "intel_realsense_sim":
                print(f"A hacky way of adding intel realsense simulated RGBD sensor bypassing clearpath_config.")
                
                sensor_description.name = "camera_0"
                sensor_description.package = "realsense2_description" # package containing the urdf file
                sensor_description.model = "d435" # name of the urdf file
                sensor_description.path = "urdf/" # folder containing the urdf file
                
                sensor_description.parameters.update({
                    'update_rate': 30,
                    'image_height': 480,
                    'image_width': 640,
                })

                # "camera_0"
                self.xacro_writer.write_comment(
                    '{0}'.format(sensor_description.name) # str
                )

                # <xacro:include filename="$(find ros2_asus_xtion_description)/urdf/asus_xtion.urdf.xacro"/>
                self.xacro_writer.write_include(
                    package=sensor_description.package, # package
                    file=sensor_description.model, # camera model
                    path=sensor_description.path # path to urdf folder
                )
                """
                <xacro:sensor_d435 name="camera_0" parent_link="fath_pivot_0_mount" update_rate="30" image_height="480" image_width="640">
                    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                </xacro:sensor_d435>
                """
                self.xacro_writer.write_macro(
                    macro='sensor_{0}'.format(sensor_description.model),
                    parameters=sensor_description.parameters,
                    blocks=XacroWriter.add_origin(
                        sensor_description.xyz, sensor_description.rpy)
                )

            else:
                # All other sensors supported by Clearpath robotics
                
                # "camera_0"
                self.xacro_writer.write_comment(
                    '{0}'.format(sensor_description.name)
                )

                # <xacro:include filename="$(find clearpath_sensors_description)/urdf/intel_realsense.urdf.xacro"/>
                self.xacro_writer.write_include(
                    package=sensor_description.package, # clearpath_sensors_description
                    file=sensor_description.model, # intel_realsense
                    path=sensor_description.path # urdf/
                )

                """
                <xacro:intel_realsense name="camera_0" parent_link="fath_pivot_0_mount" update_rate="30" image_height="480" image_width="640">
                    <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
                </xacro:intel_realsense>

                sensor_description.parameters: 
                sensor_description.parameters: {'name': 'camera_0', 'parent_link': 'fath_pivot_0_mount', 'update_rate': 30, 'image_height': 480, 'image_width': 640}

                """
                self.xacro_writer.write_macro(
                    macro='{0}'.format(sensor_description.model),
                    parameters=sensor_description.parameters,
                    blocks=XacroWriter.add_origin(
                        sensor_description.xyz, sensor_description.rpy)
                )

            self.xacro_writer.write_newline()

    def generate_manipulators(self) -> None:
        self.xacro_writer.write_comment('Manipulators')
        self.xacro_writer.write_newline()
        self.generate_arms()
        self.generate_grippers()

    def generate_arms(self) -> None:
        arms = self.clearpath_config.manipulators.get_all_arms()
        for arm in arms:
            arm_description = ManipulatorDescription(arm)

            self.xacro_writer.write_comment(
                '{0}'.format(arm_description.name)
            )

            self.xacro_writer.write_include(
                package=arm_description.package,
                file=arm_description.model,
                path=arm_description.path
            )

            self.xacro_writer.write_macro(
                macro='{0}'.format(arm_description.model),
                parameters=arm_description.parameters,
                blocks=XacroWriter.add_origin(
                    arm_description.xyz, arm_description.rpy)
            )

            self.xacro_writer.write_newline()

    def generate_grippers(self) -> None:
        arms = self.clearpath_config.manipulators.get_all_arms()
        for arm in arms:
            if not arm.gripper:
                continue
            gripper_description = ManipulatorDescription(arm.gripper)

            self.xacro_writer.write_comment(
                '{0}'.format(gripper_description.name)
            )

            self.xacro_writer.write_include(
                package=gripper_description.package,
                file=gripper_description.model,
                path=gripper_description.path,
            )

            self.xacro_writer.write_macro(
                macro='{0}'.format(gripper_description.model),
                parameters=gripper_description.parameters,
                blocks=XacroWriter.add_origin(
                    gripper_description.xyz, gripper_description.rpy)
            )

            self.xacro_writer.write_newline()

    def generate_extras(self) -> None:
        self.xacro_writer.write_comment('Extras')
        self.xacro_writer.write_newline()
        urdf_extras = self.clearpath_config.platform.extras.urdf
        if urdf_extras:
            self.xacro_writer.write_include(file=os.path.basename(urdf_extras[PackagePath.PATH]),
                                            path=os.path.dirname(
                                                urdf_extras[PackagePath.PATH]),
                                            package=urdf_extras[PackagePath.PACKAGE])
