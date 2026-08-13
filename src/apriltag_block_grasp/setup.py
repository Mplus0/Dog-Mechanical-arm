from glob import glob

from setuptools import find_packages, setup


package_name = "apriltag_block_grasp"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
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
            "probe_arm_pose = apriltag_block_grasp.tools.probe_arm_pose:main",
            "probe_arm_serial_state = apriltag_block_grasp.tools.probe_arm_serial_state:main",
            "probe_handeye_chain = apriltag_block_grasp.tools.probe_handeye_chain:main",
            "move_b_joint_safe = apriltag_block_grasp.tools.move_b_joint_safe:main",
            "trace_b_joint_motion = apriltag_block_grasp.tools.trace_b_joint_motion:main",
            "probe_handeye_pair_b = apriltag_block_grasp.tools.probe_handeye_pair_b:main",
            "probe_pnp_solutions = apriltag_block_grasp.tools.probe_pnp_solutions:main",
            "move_cartesian_fixed_orientation_safe = apriltag_block_grasp.tools.move_cartesian_fixed_orientation_safe:main",
            "probe_official_motion_interfaces = apriltag_block_grasp.tools.probe_official_motion_interfaces:main",
            "roarm_driver_node = apriltag_block_grasp.nodes.roarm_driver_node:main",
            "probe_roarm_model = apriltag_block_grasp.tools.probe_roarm_model:main",
            "move_observation_pose_safe = apriltag_block_grasp.tools.move_observation_pose_safe:main",
            "target_candidate_node = apriltag_block_grasp.nodes.target_candidate_node:main",
        ],
    },
)
