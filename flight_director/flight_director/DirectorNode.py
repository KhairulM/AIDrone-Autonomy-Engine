from drone_msgs.msg import Mission, Task
from drone_msgs.srv import LoadMission
from drone_msgs.action import ExecuteMission

from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped

from cyclonedds.domain import DomainParticipant
from cyclonedds.core import Qos, Policy, QueryCondition, SampleState
from cyclonedds.pub import DataWriter
from cyclonedds.sub import DataReader
from cyclonedds.topic import Topic

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from . import fms_interface

DirectorState = fms_interface.DirectorState
DroneState = fms_interface.DroneState


class DirectorNode(Node):
    def __init__(self):
        super().__init__("director_node")
        self.declare_parameter("mission_config_id", "latest")
        self.declare_parameter("drone_id", "127.1.1.1")
        self.declare_parameter("drone_name", "drone1")
        self.declare_parameter("drone_model", "levia1")

        self._mission_config_id = (
            self.get_parameter("mission_config_id").get_parameter_value().string_value
        )
        self._drone_name = (
            self.get_parameter("drone_name").get_parameter_value().string_value
        )
        self._drone_id = (
            self.get_parameter("drone_id").get_parameter_value().string_value
        )
        self._drone_model = (
            self.get_parameter("drone_model").get_parameter_value().string_value
        )

        self.get_logger().info(
            "Starting director node for\nDrone id: %s\nDrone name: %s\nDrone model: %s\nMission id: %s"
            % (
                self._drone_id,
                self._drone_name,
                self._drone_model,
                self._mission_config_id,
            )
        )
        # service clients
        self._load_mission_cli = self.create_client(LoadMission, "/levia/load_mission")

        # action clients
        self._execute_mission_cli = ActionClient(
            self, ExecuteMission, "/levia/execute_mission"
        )

        # topics subscription and values
        mavros_qos_profile = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

        self._flight_state_sub = self.create_subscription(
            State, "/mavros/state", self.flight_state_cb, mavros_qos_profile
        )
        self._batt_state_sub = self.create_subscription(
            BatteryState, "/mavros/battery", self.batt_state_cb, mavros_qos_profile
        )
        self._local_pose_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self.local_pose_cb,
            mavros_qos_profile,
        )
        # self._local_vel_sub = self.create_subscription(
        #     TwistStamped,
        #     "/mavros/local_position/velocity_body",
        #     self.local_vel_cb,
        #     mavros_qos_profile,
        # )

        self._fcu_state = State()
        self._battery_state = BatteryState()
        self._local_pose = PoseStamped()
        # self._local_vel = TwistStamped()

        # self._flight_connection = None
        # self._flight_armed = None
        # self._flight_mode = None
        # self._batt_percentage = None
        # self._batt_temperature = None
        # self._batt_charge = None
        # self._batt_capacity = None
        # self._batt_state = None
        # self._batt_health = None
        # self._local_pose = [None, None, None]
        # self._local_vel = [None, None, None]

        # mission execution and state
        self._state = DirectorState.INITIALIZING
        self._drone_state = DroneState.UNKNOWN
        self._mission = None
        self._current_task_index = 0
        self._current_task_progress = ""
        self._current_task_result = None
        self._current_task_error_string = ""
        self._current_task_type_string = ""
        self._rth_mission_sent = False

        self._load_mission_future = None
        self._execute_mission_future = None
        self._execute_mission_result = None

    def send_load_mission_request(self):
        req = LoadMission.Request()
        req.id = self._mission_config_id

        self._load_mission_future = self._load_mission_cli.call_async(req)
        self.get_logger().info("Load mission request sent")

    def send_execute_mission_goal(self):
        goal = ExecuteMission.Goal()

        goal.mission = self._mission
        goal.start_index = self._current_task_index

        if self._execute_mission_future != None:
            self._execute_mission_future.cancel()

        self._execute_mission_future = self._execute_mission_cli.send_goal_async(
            goal, self.execute_mission_feedback_cb
        )

        self.get_logger().info("Execute missions request sent")
        self._execute_mission_future.add_done_callback(self.execute_mission_goal_cb)

    def execute_mission_goal_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_execute_mission_result_cb)

    def execute_mission_feedback_cb(self, msg):
        feedback = msg.feedback
        self._current_task_index = feedback.current_task_index
        self._current_task_result = feedback.current_task_result
        self._current_task_error_string = feedback.current_task_error_string

        self._current_task_type_string = self.get_current_task_type_string(
            feedback.current_task.type
        )

    def get_execute_mission_result_cb(self, future):
        self._execute_mission_result = future.result().result
        self._state = DirectorState.FINISH

    def get_current_task_type_string(self, task_type):
        string = ""

        if task_type == Task.TAKEOFF:
            string = "TAKEOFF"
        if task_type == Task.LAND:
            string = "LAND"
        if task_type == Task.WAYPOINT:
            string = "WAYPOINT"
        if task_type == Task.RTH:
            string = "RTH"
        if task_type == Task.SCAN_BARCODE:
            string = "SCAN BARCODE"
        if task_type == Task.HOVER:
            string = "HOVER"
        if task_type == Task.DOCK:
            string = "DOCK"

        return string

    def flight_state_cb(self, msg):
        self._fcu_state = msg

    # TODO: add state and health as string
    def batt_state_cb(self, msg):
        self._battery_state = msg

    def local_pose_cb(self, msg):
        self._local_pose = msg

    # def local_vel_cb(self, msg):
    #     self._local_vel = msg

    def on_initializing(self):
        if not (
            self._load_mission_cli.wait_for_service(1.0)
            and self._execute_mission_cli.wait_for_server(1.0)
        ):
            self.get_logger().info("Waiting for load and execute mission server...")
        else:
            self.get_logger().info("Drone is ready")
            self._state = DirectorState.READY

    def on_ready(self):
        if (
            self._battery_state.power_supply_status
            == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        ):
            self._drone_state = DroneState.CHARGING
        else:
            self._drone_state = DroneState.IDLE

    def on_starting(self):
        if self._load_mission_future == None:
            self.send_load_mission_request()

        elif self._load_mission_future.done():
            self._mission = self._load_mission_future.result().mission
            self._state = DirectorState.EXECUTING
            self._execute_mission_future = None

            self.get_logger().info(
                "Executing loaded missions: %s" % self._mission.tasks
            )

    def on_executing(self):
        if self._execute_mission_future != None:
            return

        self.send_execute_mission_goal()
        self._drone_state = DroneState.RUNNING

    def on_pausing(self):
        if (
            self._execute_mission_future != None
            and not self._execute_mission_future.done()
        ):
            self._execute_mission_future.cancel()

        self._execute_mission_future = None

    def on_finish(self):
        self._state = DirectorState.READY
        self._drone_state = DroneState.IDLE

    def on_rth(self):
        if self._rth_mission_sent:
            return

        rth_mission = Mission()
        rth_mission.id = ""
        rth_mission.name = "EMERGENCY RTH MISSION"

        rth_task = Task()
        rth_task.type = Task.RTH

        rth_mission.tasks = [
            rth_task,
        ]

        self._mission = rth_mission
        self._current_task_index = 0
        self.send_execute_mission_goal()

        self._rth_mission_sent = True

    def on_command_start(self):
        self.get_logger().info("Start command received")

        if self._state == DirectorState.READY:
            self._state = DirectorState.STARTING
            return 0

        return -1

    def on_command_pause(self, isPause):
        self.get_logger().info("Pause command received %s" % isPause)

        if self._state == DirectorState.EXECUTING and isPause:
            self._state = DirectorState.PAUSING
            return 0
        elif self._state == DirectorState.PAUSING and not isPause:
            self._state = DirectorState.EXECUTING
            return 0

        return -1

    def on_command_restart(self):
        self.get_logger().info("Restart command received")

        if (
            self._state == DirectorState.FINISH
            or self._state == DirectorState.EXECUTING
            or self._state == DirectorState.PAUSING
        ):
            self._state = DirectorState.INITIALIZING
            self._load_mission_future = None
            return 0

        return -1

    def on_command_shutdown(self):
        self.get_logger().info("Shutdown command received")

        self._state = DirectorState.FINISH
        self._drone_state = DroneState.EMERGENCY
        return 0

    def on_command_rth(self):
        self.get_logger().info("RTH command received")

        self._state = DirectorState.RETURNING_TO_HOME
        self._drone_state = DroneState.EMERGENCY
        self._rth_mission_sent = False
        return 0


class DDSServer:
    def __init__(self, drone_id):
        self._dp = DomainParticipant(domain_id=1)

        self._status_topic = Topic(
            self._dp,
            "/drone/status",
            fms_interface.DroneStatus,
            Qos(Policy.Reliability.BestEffort, Policy.Durability.TransientLocal),
        )

        self._command_request_topic = Topic(
            self._dp,
            "/drone/command/request",
            fms_interface.DroneCommandRequest,
            Qos(Policy.Reliability.Reliable(0), Policy.Durability.Volatile),
        )

        self._command_reply_topic = Topic(
            self._dp,
            "/drone/command/reply",
            fms_interface.DroneCommandReply,
            Qos(Policy.Reliability.Reliable(0), Policy.Durability.Volatile),
        )

        self._status_dw = DataWriter(self._dp, self._status_topic)
        self._command_request_dr = DataReader(self._dp, self._command_request_topic)
        self._command_reply_dw = DataWriter(self._dp, self._command_reply_topic)
        self._command_request_filter = QueryCondition(
            self._command_request_dr,
            SampleState.NotRead,
            lambda sample: sample.drone_id == drone_id,
        )

        self._battery_health_dict = {
            0: "UNKNOWN",
            1: "GOOD",
            2: "OVERHEAT",
            3: "DEAD",
            4: "OVERVOLTAGE",
            5: "UNSPEC_FAILURE",
            6: "COLD",
            7: "WATCHDOG_TIMER_EXPIRE",
            8: "SAFETY_TIMER_EXPIRE",
        }

        self._battery_status_dict = {
            0: "UNKNOWN",
            1: "CHARGING",
            2: "DISCHARGING",
            3: "NOT_CHARGING",
            4: "FULL",
        }

    def process_command_request(
        self, node: DirectorNode, command_request: fms_interface.DroneCommandRequest
    ):
        corr_id = command_request.corr_id
        command = command_request.command
        result = 0

        if command == fms_interface.DroneCommand.START:
            result = node.on_command_start()
        elif command == fms_interface.DroneCommand.PAUSE:
            result = node.on_command_pause(True)
        elif command == fms_interface.DroneCommand.RESUME:
            result = node.on_command_pause(False)
        elif command == fms_interface.DroneCommand.SHUTDOWN:
            result = node.on_command_shutdown()
        elif command == fms_interface.DroneCommand.RESTART:
            result = node.on_command_restart()
        elif command == fms_interface.DroneCommand.RTH:
            result = node.on_command_rth()

        reply = fms_interface.DroneCommandReply(node._drone_id, corr_id, result)

        return reply

    def spin_once(self, node: DirectorNode):
        local_position = node._local_pose.pose.position

        drone_status_sample = fms_interface.DroneStatus(
            node._drone_id,
            node._drone_name,
            node._drone_model,
            node._mission.id,
            node._current_task_index,
            node._fcu_state.mode,
            fms_interface.LocalPosition(
                local_position.x, local_position.y, local_position.z
            ),
            node._drone_state,
            node._state,
            fms_interface.BatteryStatus(
                node._battery_state.percentage,
                node._battery_state.temperature,
                node._battery_state.charge,
                node._battery_state.capacity,
                self._battery_status_dict[node._battery_state.power_supply_status],
                self._battery_health_dict[node._battery_state.power_supply_health],
            ),
        )

        self._status_dw.write(drone_status_sample)
        # node.get_logger().info(
        #     "Sent DDS drone status: %s\n" % drone_status_sample
        # )

        command_request = self._command_request_dr.read(
            N=1, condition=self._command_request_filter
        )

        if len(command_request) > 0:
            command_request = command_request[0]

            # node.get_logger().info(
            #     "Received command request: %s" % command_request
            # )

            command_reply = self.process_command_request(node, command_request)

            self._command_reply_dw.write(command_reply)


def main(args=None):
    rclpy.init(args=args)

    director_node = DirectorNode()
    dds_server = DDSServer(director_node._drone_id)

    while rclpy.ok():
        rclpy.spin_once(director_node)
        dds_server.spin_once(director_node)

        if director_node._state == DirectorState.INITIALIZING:
            director_node.on_initializing()
        elif director_node._state == DirectorState.READY:
            director_node.on_ready()
        elif director_node._state == DirectorState.STARTING:
            director_node.on_starting()
        elif director_node._state == DirectorState.EXECUTING:
            director_node.on_executing()
        elif director_node._state == DirectorState.PAUSING:
            director_node.on_pausing()
        elif director_node._state == DirectorState.FINISH:
            director_node.on_finish()
        elif director_node._state == DirectorState.RETURNING_TO_HOME:
            director_node.on_rth()

    director_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
