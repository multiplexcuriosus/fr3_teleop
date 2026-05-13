#!/usr/bin/env python3

import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Trigger
from std_srvs.srv import SetBool
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


class RecordManagerNode(Node):
    def __init__(self):
        super().__init__("record_manager")

        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter("target_dir", "data/bags")
        self.declare_parameter("prefix", "")
        self.declare_parameter("bag_name", "")

        self.declare_parameter("record_raw_events", True)
        self.declare_parameter("openmv_node_name", "/openmv_event_cam")
        self.declare_parameter("openmv_start_service", "/openmv_cam/start_raw_event_recording")
        self.declare_parameter("openmv_stop_service", "/openmv_cam/stop_raw_event_recording")
        self.declare_parameter("raw_event_suffix", "_raw_events.h5")

        self.declare_parameter("topics", [
            "/episode/control",
            "/camera/camera/color/camera_info",
            "/cartesian_cmd/twist",
            "/teleop/gripper_state_cmd",
            "/camera/camera/color/image_raw",
            "/openmv_cam/image",
            "/joint_states",
            "/chatter" #debug
        ])
        self.declare_parameter("topics_to_record", rclpy.Parameter.Type.STRING_ARRAY)

        self.declare_parameter("bag_storage", "sqlite3")
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter("bag_shutdown_timeout_sec", 10.0)
        self.declare_parameter("enforce_topic_presence", True)
        self.declare_parameter("debug_bypass_topic_presence", False)

        self.bag_proc = None
        self.current_bag_path = None
        self.current_raw_event_path = None
        self.current_session_dir = None
        self.debug_bypass_topic_presence = self._get_bool_param("debug_bypass_topic_presence")

        self.start_srv = self.create_service(
            Trigger,
            "/record_manager/start_recording",
            self._handle_start_recording,
            callback_group=self.cb_group,
        )
        self.stop_srv = self.create_service(
            Trigger,
            "/record_manager/stop_recording",
            self._handle_stop_recording,
            callback_group=self.cb_group,
        )
        self.set_debug_bypass_srv = self.create_service(
            SetBool,
            "/record_manager/set_debug_bypass_topic_presence",
            self._handle_set_debug_bypass_topic_presence,
            callback_group=self.cb_group,
        )

        self.get_logger().info("Record manager ready")
        self.get_logger().info("Services:")
        self.get_logger().info("  /record_manager/start_recording")
        self.get_logger().info("  /record_manager/stop_recording")
        self.get_logger().info("  /record_manager/set_debug_bypass_topic_presence")
        self.get_logger().info(
            "Topic presence gate: "
            f"enforce={self._get_bool_param('enforce_topic_presence')}, "
            f"debug_bypass={self.debug_bypass_topic_presence}"
        )

    def _get_str_param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_bool_param(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _get_double_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _get_str_array_param(self, name):
        return list(self.get_parameter(name).get_parameter_value().string_array_value)

    def _get_topics_to_record(self):
        preferred = [
            topic.strip() for topic in self._get_str_array_param("topics_to_record") if topic.strip()
        ]
        if preferred:
            return preferred
        return [topic.strip() for topic in self._get_str_array_param("topics") if topic.strip()]

    def _make_session_paths(self):
        target_dir = Path(self._get_str_param("target_dir")).expanduser()
        prefix = self._get_str_param("prefix")
        bag_name = self._get_str_param("bag_name")

        target_dir.mkdir(parents=True, exist_ok=True)

        if bag_name:
            session_name = bag_name
        else:
            timestamp = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            session_name = f"{prefix}_{timestamp}" if prefix else f"recording_{timestamp}"

        session_dir = (target_dir / session_name).resolve()
        bag_path = session_dir / f"{session_name}_bag"
        raw_event_path = session_dir / f"{session_name}_raw_events.h5"

        if session_dir.exists():
            raise RuntimeError(f"Session directory already exists: {session_dir}")

        session_dir.mkdir(parents=True, exist_ok=False)

        return session_dir, bag_path, raw_event_path

    def _wait_for_service(self, client, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() - start > timeout_sec:
                return False
        return True

    def _call_trigger_service(self, service_name, timeout_sec):
        client = self.create_client(
            Trigger,
            service_name,
            callback_group=self.cb_group,
        )

        if not self._wait_for_service(client, timeout_sec):
            self.destroy_client(client)
            raise RuntimeError(f"Service not available: {service_name}")

        future = client.call_async(Trigger.Request())

        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > timeout_sec:
                self.destroy_client(client)
                raise RuntimeError(f"Service call timed out: {service_name}")
            time.sleep(0.02)

        result = future.result()
        self.destroy_client(client)

        if result is None:
            raise RuntimeError(f"Service call failed: {service_name}")

        if not result.success:
            raise RuntimeError(f"Service returned failure: {service_name}")

        return result

    def _set_openmv_raw_event_path(self, output_path, timeout_sec):
        # Prefer the namespace implied by the raw-event start service (e.g.
        # /openmv_cam/start_raw_event_recording -> /openmv_cam/set_parameters),
        # then fall back to legacy openmv_node_name-based lookup.
        candidates = []

        start_service = self._get_str_param("openmv_start_service").strip()
        if start_service:
            start_service_path = Path(start_service)
            parent_ns = str(start_service_path.parent)
            if parent_ns and parent_ns != ".":
                candidates.append(f"{parent_ns}/set_parameters")

        openmv_node_name = self._get_str_param("openmv_node_name").strip()
        if openmv_node_name:
            candidates.append(f"{openmv_node_name}/set_parameters")

        # Deduplicate while preserving order.
        unique_candidates = []
        for service_name in candidates:
            if service_name not in unique_candidates:
                unique_candidates.append(service_name)

        if not unique_candidates:
            raise RuntimeError("No candidate OpenMV parameter services available")

        client = None
        selected_service_name = None
        for service_name in unique_candidates:
            probe_client = self.create_client(
                SetParameters,
                service_name,
                callback_group=self.cb_group,
            )
            if self._wait_for_service(probe_client, timeout_sec):
                client = probe_client
                selected_service_name = service_name
                break
            self.destroy_client(probe_client)

        if client is None:
            raise RuntimeError(
                "Parameter service not available. Tried: "
                + ", ".join(unique_candidates)
            )

        param = Parameter()
        param.name = "raw_event_output_path"
        param.value = ParameterValue()
        param.value.type = ParameterType.PARAMETER_STRING
        param.value.string_value = str(output_path)

        req = SetParameters.Request()
        req.parameters = [param]

        future = client.call_async(req)

        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > timeout_sec:
                self.destroy_client(client)
                raise RuntimeError(f"Parameter set timed out: {selected_service_name}")
            time.sleep(0.02)

        result = future.result()
        self.destroy_client(client)

        if result is None or not result.results:
            raise RuntimeError("Failed to set raw_event_output_path")

        if not result.results[0].successful:
            reason = result.results[0].reason
            raise RuntimeError(f"Failed to set raw_event_output_path: {reason}")

        self.get_logger().info(
            f"Set raw_event_output_path via {selected_service_name}: {output_path}"
        )

    def _missing_required_topics(self):
        topics = self._get_topics_to_record()
        missing = []
        for topic in topics:
            publishers = self.get_publishers_info_by_topic(topic)
            if len(publishers) == 0:
                missing.append(topic)
        return missing

    def _is_topic_presence_gate_enabled(self):
        return self._get_bool_param("enforce_topic_presence") and not self.debug_bypass_topic_presence

    def _handle_set_debug_bypass_topic_presence(self, request, response):
        self.debug_bypass_topic_presence = bool(request.data)
        state = "ON" if self.debug_bypass_topic_presence else "OFF"
        self.get_logger().warn(f"Debug bypass topic presence set to {state}")
        response.success = True
        response.message = f"debug_bypass_topic_presence={self.debug_bypass_topic_presence}"
        return response

    def _start_bag_recording(self, bag_path):
        topics = self._get_topics_to_record()
        storage = self._get_str_param("bag_storage")

        if not topics:
            raise RuntimeError("No topics configured for rosbag recording")

        if bag_path.exists():
            raise RuntimeError(f"Bag path already exists: {bag_path}")

        cmd = [
            "ros2", "bag", "record",
            "-o", str(bag_path),
            "-s", storage,
            *topics,
        ]

        self.get_logger().info("Starting rosbag:")
        self.get_logger().info(" ".join(cmd))

        # start_new_session=True lets us SIGINT the whole process group cleanly.
        self.bag_proc = subprocess.Popen(
            cmd,
            stdout=None,
            stderr=None,
            start_new_session=True,
        )

    def _stop_bag_recording(self):
        if self.bag_proc is None:
            return

        timeout_sec = self._get_double_param("bag_shutdown_timeout_sec")

        if self.bag_proc.poll() is None:
            self.get_logger().info("Stopping rosbag with SIGINT")
            os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGINT)

            try:
                self.bag_proc.wait(timeout=timeout_sec)
            except subprocess.TimeoutExpired:
                self.get_logger().warn("rosbag did not stop after SIGINT; sending SIGTERM")
                os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGTERM)
                try:
                    self.bag_proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    self.get_logger().warn("rosbag did not stop after SIGTERM; sending SIGKILL")
                    os.killpg(os.getpgid(self.bag_proc.pid), signal.SIGKILL)
                    self.bag_proc.wait(timeout=3.0)

        self.bag_proc = None

    def _handle_start_recording(self, request, response):
        del request

        if self.bag_proc is not None and self.bag_proc.poll() is None:
            response.success = False
            response.message = ""
            self.get_logger().warn("Recording already active")
            return response

        service_timeout = self._get_double_param("service_timeout_sec")
        record_raw_events = self._get_bool_param("record_raw_events")
        start_raw_service = self._get_str_param("openmv_start_service")

        if self._is_topic_presence_gate_enabled():
            missing_topics = self._missing_required_topics()
            if missing_topics:
                reason = (
                    "Topic presence check failed; missing publishers on: "
                    + ", ".join(missing_topics)
                )
                self.get_logger().warn(reason)
                response.success = False
                response.message = reason
                return response

        session_dir, bag_path, raw_event_path = self._make_session_paths()
        self.current_session_dir = session_dir
        self.current_bag_path = bag_path
        self.current_raw_event_path = raw_event_path

        self.get_logger().info(f"Recording session: {session_dir}")
        self.get_logger().info(f"Rosbag: {bag_path}")
        self.get_logger().info(f"Raw events: {raw_event_path}")

        try:
            if record_raw_events:
                if raw_event_path.exists():
                    raise RuntimeError(f"Raw event file already exists: {raw_event_path}")

                self._set_openmv_raw_event_path(raw_event_path, service_timeout)

                self._call_trigger_service(start_raw_service, service_timeout)

            self._start_bag_recording(bag_path)

            self.current_bag_path = bag_path
            self.current_raw_event_path = raw_event_path if record_raw_events else None

        except Exception as e:
            self.get_logger().error(f"Failed to start recording: {e}")

            # Try to clean up raw event recording if it already started.
            if record_raw_events:
                try:
                    self._call_trigger_service(
                        self._get_str_param("openmv_stop_service"),
                        service_timeout,
                    )
                except Exception as stop_e:
                    self.get_logger().warn(f"Cleanup stop_raw_event_recording failed: {stop_e}")

            response.success = False
            response.message = str(e)
            return response

        self.get_logger().info(f"Recording started: {bag_path}")
        if record_raw_events:
            self.get_logger().info(f"Raw events: {raw_event_path}")

        response.success = True
        response.message = ""
        return response

    def _handle_stop_recording(self, request, response):
        del request

        service_timeout = self._get_double_param("service_timeout_sec")
        record_raw_events = self._get_bool_param("record_raw_events")
        stop_raw_service = self._get_str_param("openmv_stop_service")

        self.get_logger().info(f"Recording stopped: {self.current_session_dir}")
        self.get_logger().info(f"Rosbag: {self.current_bag_path}")
        self.get_logger().info(f"Raw event file: {self.current_raw_event_path}")

        if self.bag_proc is None:
            self.get_logger().warn("No active rosbag process")
            # Still try to stop raw event recording if requested.
            if record_raw_events:
                try:
                    self._call_trigger_service(stop_raw_service, service_timeout)
                except Exception as e:
                    self.get_logger().warn(f"stop_raw_event_recording failed: {e}")

            response.success = False
            response.message = ""
            return response

        try:
            # Stop bag first so episode/topic recording ends cleanly.
            self._stop_bag_recording()

            if record_raw_events:
                self._call_trigger_service(stop_raw_service, service_timeout)

        except Exception as e:
            self.get_logger().error(f"Failed to stop recording cleanly: {e}")
            response.success = False
            response.message = ""
            return response

        self.get_logger().info(f"Recording stopped: {self.current_bag_path}")
        if self.current_raw_event_path is not None:
            self.get_logger().info(f"Raw event file: {self.current_raw_event_path}")

        self.current_bag_path = None
        self.current_raw_event_path = None

        response.success = True
        response.message = ""
        return response

    def destroy_node(self):
        if self.bag_proc is not None and self.bag_proc.poll() is None:
            self.get_logger().warn("Node shutting down while recording is active")
            try:
                self._stop_bag_recording()
            except Exception as e:
                self.get_logger().warn(f"Failed to stop bag on shutdown: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RecordManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()