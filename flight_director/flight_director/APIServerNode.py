from drone_msgs.msg import Mission, Task
from drone_msgs.srv import LoadMission, LoadConfiguration

import rclpy
from rclpy.node import Node

import requests


class APIServerNode(Node):
    def __init__(self):
        super().__init__("api_server_node")
        self.declare_parameter("api_server_url", "http://localhost:6868/api/v1")

        self._api_server_url = (
            self.get_parameter("api_server_url").get_parameter_value().string_value
        )

        self._load_mission_srv = self.create_service(
            LoadMission, "load_mission", self.load_mission_cb
        )

        self._load_configuration_srv = self.create_service(
            LoadConfiguration, "load_configuration", self.load_configuration_cb
        )

        self._mission_config_url = "%s/config" % self._api_server_url

    def load_mission_cb(self, request, response):
        self.get_logger().info(
            "Incoming load mission request with id: %s" % (request.id)
        )

        test_mission = Mission()
        test_mission.id = "TEST_MISSION_1"
        test_mission.name = "TEST MISSION 1"

        takeoff_task = Task()
        takeoff_task.type = Task.TAKEOFF
        takeoff_task.float_args = [5.0]

        test_mission.tasks = [takeoff_task]

        response.mission = test_mission
        response.success = True

        try:
            r = requests.get("%s/%s" % (self._mission_config_url, request.id))

            if r.status_code != 200:
                raise Exception("Get mission config failed, %d Error" % r.status_code)
        except Exception as e:
            self.get_logger().error(str(e))
            response.error_string = str(e)
        else:
            # parsing mission configuration to list of missions
            mission_configuration = r.json()

        return response

    def load_configuration_cb(self, request, response):
        self.get_logger().info(
            "Incoming load configuration request with id: %s" % (request.id)
        )

        return response


def main():
    rclpy.init()

    api_node = APIServerNode()
    rclpy.spin(api_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
