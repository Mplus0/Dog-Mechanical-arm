#!/usr/bin/env python3
"""Inspect official RoArm ROS 2 service interfaces without creating clients."""

import json
from typing import Any, Dict, Optional, Type

import rclpy
from rclpy.node import Node


def message_fields(message_type: Type[Any]) -> Dict[str, str]:
    message = message_type()
    if hasattr(message, "get_fields_and_field_types"):
        return dict(message.get_fields_and_field_types())
    slots = getattr(message, "__slots__", ())
    return {str(slot).lstrip("_"): "unknown" for slot in slots}


def service_description(service_type: Type[Any]) -> Dict[str, Any]:
    return {
        "python_type": f"{service_type.__module__}.{service_type.__name__}",
        "request_fields": message_fields(service_type.Request),
        "response_fields": message_fields(service_type.Response),
    }


class OfficialMotionInterfaceProbe(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_official_motion_interface_probe")

    def inspect(self) -> Dict[str, Any]:
        report: Dict[str, Any] = {
            "tool": "apriltag_block_grasp.probe_official_motion_interfaces",
            "read_only": True,
            "service_clients_created": False,
            "service_requests_sent": 0,
            "serial_opened": False,
            "camera_opened": False,
            "motion_commands_enabled": False,
            "interfaces": {},
            "ros_graph_services": {},
            "summary": {"valid": False},
        }
        try:
            from roarm_msgs.srv import GetPoseCmd, MoveJointCmd, MoveLineCmd
        except Exception as exc:
            report["summary"] = {
                "valid": False,
                "reason": "roarm_service_import_failed",
                "error": f"{type(exc).__name__}: {exc}",
            }
            return report

        service_types = {
            "/get_pose_cmd": GetPoseCmd,
            "/move_joint_cmd": MoveJointCmd,
            "/move_line_cmd": MoveLineCmd,
        }
        report["interfaces"] = {
            name: service_description(service_type)
            for name, service_type in service_types.items()
        }
        graph = dict(self.get_service_names_and_types())
        report["ros_graph_services"] = {
            name: {
                "available": name in graph,
                "advertised_types": list(graph.get(name, [])),
            }
            for name in service_types
        }
        available = [name for name in service_types if name in graph]
        report["summary"] = {
            "valid": True,
            "reason": "interfaces_inspected_without_calls",
            "imported_interface_count": len(service_types),
            "available_service_count": len(available),
            "available_services": available,
            "motion_command_sent": False,
        }
        return report


def main(args=None) -> int:
    rclpy.init(args=args)
    node: Optional[OfficialMotionInterfaceProbe] = None
    try:
        node = OfficialMotionInterfaceProbe()
        report = node.inspect()
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["summary"].get("valid", False) else 1
    except Exception as exc:
        print(
            json.dumps(
                {
                    "tool": "apriltag_block_grasp.probe_official_motion_interfaces",
                    "read_only": True,
                    "service_clients_created": False,
                    "service_requests_sent": 0,
                    "serial_opened": False,
                    "motion_commands_enabled": False,
                    "summary": {
                        "valid": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    },
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
