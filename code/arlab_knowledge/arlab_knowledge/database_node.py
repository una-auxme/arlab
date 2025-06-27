from rclpy.callback_groups import ReentrantCallbackGroup
from arlab_knowledge_interfaces.srv import GetEntities
from arlab_knowledge_interfaces.srv import GetPoseAndReference
from arlab_knowledge_interfaces.srv import GetShape
from arlab_knowledge_interfaces.srv import DoorGetOpen
from arlab_knowledge_interfaces.srv import DoorGetWidth
from arlab_knowledge_interfaces.srv import GetDescription
from arlab_knowledge_interfaces.srv import AddEntity
from arlab_knowledge_interfaces.msg import EntityType
from arlab_knowledge_interfaces.msg import Result


import rclpy
from rclpy.node import Node

prefix = "/arlab"


class DatabaseNode(Node):
    """This node provides a service that adds two ints

    The custom service definition can be found in the
    arlab_template_interfaces.srv folder
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            GetEntities,
            f"{prefix}/get_entities_callback",
            self.get_entities_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            GetShape,
            "/get_shape_callback",
            self.get_shape_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            DoorGetOpen,
            "/get_door_open_callback",
            self.get_door_open_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            DoorGetWidth,
            "/get_door_width_callback",
            self.get_door_width_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            GetPoseAndReference,
            "/get_pose_and_reference_callback",
            self.get_pose_and_reference_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            GetDescription,
            "/get_description_callback",
            self.get_description_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            AddEntity,
            "/add_entity_callback ",
            self.add_entity_callback,
            callback_group=self.reentrant_callback_group,
        )

    async def get_entities_callback(
        self, request: GetEntities.Request, response: GetEntities.Response
    ):
        ### TODO implement the function once the database exists
        # request an datenbank weiterleiten
        # answer = antwort von datenbank bestehen aus EntitiyType und Result
        answer = 1
        if answer:
            # response.entities = answer.EntityType
            response.entities = 0
        self.get_logger().info("Incoming request:")
        return response

    async def get_shape_callback(
        self, request: GetShape.Request, response: GetShape.Response
    ):
        ### TODO implement the function once the database exists
        return response

    async def get_door_open_callback(
        self, request: DoorGetOpen.Request, response: DoorGetOpen.Response
    ):
        ### TODO implement the function once the database exists
        return response

    async def get_door_width_callback(
        self, request: DoorGetWidth.Request, response: DoorGetWidth.Response
    ):
        ### TODO implement the function once the database exists
        return response

    async def get_pose_and_reference_callback(
        self,
        request: GetPoseAndReference.Request,
        response: GetPoseAndReference.Response,
    ):
        ### TODO implement the function once the database exists
        return response

    async def get_description_callback(
        self, request: GetDescription.Request, response: GetDescription.Response
    ):
        ### TODO implement the function once the database exists
        return response

    async def add_entity_callback(
        self, request: AddEntity.Request, response: AddEntity.Response
    ):
        pass


def main(args=None):
    rclpy.init(args=args)

    database_node = DatabaseNode()
    try:
        database_node.get_logger().info(
            "Beginning database_node, shut down with CTRL-C"
        )
        rclpy.spin(database_node)
    except KeyboardInterrupt:
        database_node.get_logger().info(
            "Keyboard interrupt, shutting down database_node.\n"
        )
    database_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
