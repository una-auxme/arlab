import os
import asyncio
from contextlib import asynccontextmanager

import sqlalchemy
from sqlalchemy.ext.asyncio import async_sessionmaker, create_async_engine, AsyncSession
from sqlalchemy.exc import SQLAlchemyError, DBAPIError
from sqlalchemy import select
from sqlalchemy.orm import selectinload

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Pose

from arlab_knowledge_interfaces.srv import GetEntities
from arlab_knowledge_interfaces.srv import GetPose
from arlab_knowledge_interfaces.srv import GetShape
from arlab_knowledge_interfaces.srv import DoorGetOpen
from arlab_knowledge_interfaces.srv import DoorGetWidth
from arlab_knowledge_interfaces.srv import GetDescription
from arlab_knowledge_interfaces.srv import AddEntity
from arlab_knowledge_interfaces.msg import EntityType
from arlab_knowledge_interfaces.msg import Result

from arlab_knowledge.db.base import Base
from arlab_knowledge.db.entities.entity import Entity
from arlab_knowledge.db.entities.shape import Shape
from arlab_knowledge.db.entities.human import Human
from arlab_knowledge.db.entities.furniture import (
    Furniture,
    Cupboard,
    Door,
    Table,
    Shelf,
)
from arlab_knowledge.db.entities.pickable import Pickable
from arlab_knowledge.db.ros_adapters.pose import PoseData
from arlab_knowledge.db.ros_adapters.time import TimeData

from arlab_asyncio_executor.executors import AsyncIOExecutor

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
            "postgresql+asyncpg",
            username=database_user,
            password=database_password,
            host=host,
            port=int(port),
            database=database_name,
        )

        self.db_engine = create_async_engine(self.db_url, echo=True)
        self.db_sessionmaker = async_sessionmaker(
            self.db_engine, expire_on_commit=False
        )

        self.reentrant_callback_group = ReentrantCallbackGroup()

    @classmethod
    async def create(cls) -> "DatabaseNode":
        node = cls()
        async with node.db_engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
        node.init_services()
        return node

    def init_services(self):
        self.create_service(
            GetEntities,
            f"{prefix}/get_entities",
            self.get_entities_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetShape,
            f"{prefix}/get_shape",
            self.get_shape_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            DoorGetOpen,
            f"{prefix}/door_get_open",
            self.door_get_open_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            DoorGetWidth,
            f"{prefix}/door_get_width",
            self.door_get_width_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetPose,
            f"{prefix}/get_pose",
            self.get_pose_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetDescription,
            f"{prefix}/get_description",
            self.get_description_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            AddEntity,
            f"{prefix}/add_entity",
            self.add_entity_callback,
            callback_group=self.reentrant_callback_group,
        )

    @asynccontextmanager
    async def Session(self, response):
        response.result.result_type = Result.SUCCESS
        try:
            async with self.db_sessionmaker() as session:
                yield session
        except DBAPIError as e:
            response.result.error = str(e)
            response.result.result_type = Result.ERROR_DBAPI
        except SQLAlchemyError as e:
            response.result.error = str(e)
            response.result.result_type = Result.ERROR_SQL

    async def get_entities_callback(
        self, request: GetEntities.Request, response: GetEntities.Response
    ):
        async with self.Session(response) as session:
            if request.entity_type == EntityType.ENTITY:
                entity_class = Entity
            elif request.entity_type == EntityType.HUMAN:
                entity_class = Human
            elif request.entity_type == EntityType.CUPBOARD:
                entity_class = Cupboard
            elif request.entity_type == EntityType.PICKABLE:
                entity_class = Pickable
            elif request.entity_type == EntityType.DOOR:
                entity_class = Door
            elif request.entity_type == EntityType.FURNITURE:
                entity_class = Furniture
            elif request.entity_type == EntityType.SHELF:
                entity_class = Shelf
            elif request.entity_type == EntityType.TABLE:
                entity_class = Table
            else:
                entity_class = None

            if entity_class is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
            else:
                stmt = select(entity_class.id)
                result = await session.execute(stmt)
                entities = result.scalars().all()
                response.entities = entities
        self.get_logger().info("Incoming request:")
        return response

    async def get_shape_callback(
        self, request: GetShape.Request, response: GetShape.Response
    ):
        async with self.Session(response) as session:
            stmt = select(Shape).where(Shape.entity_id == request.entityid)
            result = await session.execute(stmt)
            shape = result.scalar_one_or_none()
            if shape is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
            else:
                response.shape = result
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
        async with self.Session(response) as session:
            stmt = select(Pose).where(Entity.id == request.entityid)
            result = await session.execute(stmt)
            pose = result.scalar_one_or_none()
            if pose is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
            else:
                response.pose = result
        return response

    async def get_description_callback(
        self, request: GetDescription.Request, response: GetDescription.Response
    ):
        async with self.Session(response) as session:
            stmt = select(Entity.description).where(Entity.id == request.entityid)
            result = await session.execute(stmt)
            description = result.scalar_one_or_none()
            if description is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
            else:
                response.description = result
        return response

    async def add_entity_callback(
        self, request: AddEntity.Request, response: AddEntity.Response
    ):
        async with self.Session(response) as session:
            entity = Entity(
                description=request.description,
                pose=PoseData(request.pose),
                frame_id=request.pose_reference_frame,
                stamp=TimeData(request.stamp),
            )
            async with session.begin():
                session.add(entity)
            response.entityid = entity.id
        return response

    def destroy_node(self):
        asyncio.run(self.db_engine.dispose())
        super().destroy_node()


async def ros_loop():
    asyncio_loop = asyncio.get_running_loop()
    executor = AsyncIOExecutor(asyncio_loop)
    database_node = await DatabaseNode.create()
    executor.add_node(database_node)

    try:
        database_node.get_logger().info(
            "Beginning database_node, shut down with CTRL-C"
        )
        await asyncio.to_thread(executor.spin)
    except KeyboardInterrupt:
        database_node.get_logger().info(
            "Keyboard interrupt, shutting down database_node.\n"
        )
    database_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    asyncio.run(ros_loop())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
