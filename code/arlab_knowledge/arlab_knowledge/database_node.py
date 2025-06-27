import os

import sqlalchemy
from sqlalchemy import create_engine
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from arlab_knowledge_interfaces.srv import GetEntities
from arlab_knowledge_interfaces.srv import GetPose
from arlab_knowledge_interfaces.srv import GetShape
from arlab_knowledge_interfaces.srv import DoorGetOpen
from arlab_knowledge_interfaces.srv import DoorGetWidth
from arlab_knowledge_interfaces.srv import GetDescription
from arlab_knowledge_interfaces.srv import AddEntity
from arlab_knowledge_interfaces.msg import EntityType
from arlab_knowledge_interfaces.msg import Result

from arlab_knowledge import Base
from arlab_knowledge.entities.entity import Entity

prefix = "/arlab/knowledge"


class DatabaseNode(Node):
    """This node provides a service that adds two ints

    The custom service definition can be found in the
    arlab_template_interfaces.srv folder
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        database_host = os.getenv("KNOWLEDGE_BASE_POSTGRES_HOST")
        database_name = os.getenv("KNOWLEDGE_BASE_POSTGRES_DB")
        database_user = os.getenv("KNOWLEDGE_BASE_POSTGRES_USER")
        database_password = os.getenv("KNOWLEDGE_BASE_POSTGRES_PASSWORD")
        if (
            database_host is None
            or database_name is None
            or database_user is None
            or database_password is None
        ):
            msg = "Database environment variables not set"
            self.get_logger().error(msg)
            raise RuntimeError(msg)

        host, port = (
            database_host.split(":") if ":" in database_host else (database_host, 5432)
        )
        self.db_url = sqlalchemy.engine.URL.create(
            "postgresql+psycopg2",
            username=database_user,
            password=database_password,
            host=host,
            port=int(port),
            database=database_name,
        )

        self.db_engine = create_engine(url=self.db_url, echo=True)
        Base.metadata.create_all(self.db_engine)

        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.srv = self.create_service(
            GetEntities,
            f"{prefix}/get_entities",
            self.get_entities_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            GetShape,
            f"{prefix}/get_shape",
            self.get_shape_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            DoorGetOpen,
            f"{prefix}/door_get_open",
            self.door_get_open_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            DoorGetWidth,
            f"{prefix}/door_get_width",
            self.door_get_width_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            GetPose,
            f"{prefix}/get_pose",
            self.get_pose_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            GetDescription,
            f"{prefix}/get_description",
            self.get_description_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.srv = self.create_service(
            AddEntity,
            f"{prefix}/add_entity",
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

    async def door_get_open_callback(
        self, request: DoorGetOpen.Request, response: DoorGetOpen.Response
    ):
        ### TODO implement the function once the database exists
        return response

    async def door_get_width_callback(
        self, request: DoorGetWidth.Request, response: DoorGetWidth.Response
    ):
        ### TODO implement the function once the database exists
        return response

    async def get_pose_callback(
        self,
        request: GetPose.Request,
        response: GetPose.Response,
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
