from glob import glob
from setuptools import find_packages, setup

package_name = "red_block_grasp_mplus0"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/docs", glob("docs/*.yaml") + glob("docs/*.md")),
        ("share/" + package_name + "/handeye", glob("handeye/*.json")),
        ("share/" + package_name + "/models", glob("models/*.pt")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sunrise",
    maintainer_email="sunrise@example.com",
    description="ROS2 red block grasp project for RoArm-M3",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_camera_node = red_block_grasp_mplus0.nodes.yolo_camera_node:main",
            "target_localizer_node = red_block_grasp_mplus0.nodes.target_localizer_node:main",
            "roarm_driver_node = red_block_grasp_mplus0.nodes.roarm_driver_node:main",
            "task_manager_node = red_block_grasp_mplus0.nodes.task_manager_node:main",
            "visual_servo_task_node = red_block_grasp_mplus0.nodes.visual_servo_task_node:main",
            "competition_arm_task_node = red_block_grasp_mplus0.nodes.competition_arm_task_node:main",
            "dog_arm_tcp_server_node = red_block_grasp_mplus0.nodes.dog_arm_tcp_server_node:main",
            "open_loop_grasp_task_node = red_block_grasp_mplus0.nodes.open_loop_grasp_task_node:main",
            "execution_logger_node = red_block_grasp_mplus0.nodes.execution_logger_node:main",
            "calibrate_red_threshold = red_block_grasp_mplus0.tools.calibrate_red_threshold:main",
            "calibrate_place_pose = red_block_grasp_mplus0.tools.calibrate_place_pose:main",
        ],
    },
)
