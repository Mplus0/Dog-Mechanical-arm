from setuptools import find_packages, setup


package_name = "apriltag_block_grasp"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="sunrise@example.com",
    description="Standalone AprilTag-guided block grasp package for RoArm-M3.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "check_environment = apriltag_block_grasp.tools.check_environment:main",
            "color_camera_check_node = apriltag_block_grasp.nodes.color_camera_check_node:main",
            "apriltag_detection_2d_node = apriltag_block_grasp.nodes.apriltag_detection_2d_node:main",
            "probe_color_calibration = apriltag_block_grasp.tools.probe_color_calibration:main",
            "apriltag_pnp_node = apriltag_block_grasp.nodes.apriltag_pnp_node:main",
            "probe_rgbd_alignment = apriltag_block_grasp.tools.probe_rgbd_alignment:main",
            "probe_pnp_depth_consistency = apriltag_block_grasp.tools.probe_pnp_depth_consistency:main",
        ],
    },
)
