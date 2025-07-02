import os
import asyncio
from contextlib import asynccontextmanager

import sqlalchemy
from sqlalchemy.ext.asyncio import async_sessionmaker, create_async_engine
from sqlalchemy.exc import SQLAlchemyError, DBAPIError
from sqlalchemy.orm import joinedload
from sqlalchemy import select
from sqlalchemy import delete

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

from arlab_knowledge_interfaces.srv import UpdEntity
from arlab_knowledge_interfaces.srv import UpdCupboard
from arlab_knowledge_interfaces.srv import UpdDoor
from arlab_knowledge_interfaces.srv import UpdFurniture
from arlab_knowledge_interfaces.srv import UpdHuman
from arlab_knowledge_interfaces.srv import UpdPickable
from arlab_knowledge_interfaces.srv import UpdShelf
from arlab_knowledge_interfaces.srv import UpdTable
from arlab_knowledge_interfaces.srv import UpdPose
from arlab_knowledge_interfaces.srv import UpdShape
from arlab_knowledge_interfaces.srv import DelEntities
from arlab_knowledge_interfaces.srv import GetReference


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

        self.create_service(
            FurnitureGetPickable,
            f"{prefix}/furniture_get_pickable",
            self.furniture_get_pickable_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            PickableGetFurniture,
            f"{prefix}/pickable_get_furniture",
            self.pickable_get_furniture_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            CupboardGetShelf,
            f"{prefix}/cupboard_get_shelf",
            self.cupboard_get_shelf_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            ShelfGetCupboard,
            f"{prefix}/shelf_get_cupboard",
            self.shelf_get_cupboard_callback,
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
            if request.entity_type.entity_type == EntityType.ENTITY:
                entity_class = Entity
            elif request.entity_type.entity_type == EntityType.HUMAN:
                entity_class = Human
            elif request.entity_type.entity_type == EntityType.CUPBOARD:
                entity_class = Cupboard
            elif request.entity_type.entity_type == EntityType.PICKABLE:
                entity_class = Pickable
            elif request.entity_type.entity_type == EntityType.DOOR:
                entity_class = Door
            elif request.entity_type.entity_type == EntityType.FURNITURE:
                entity_class = Furniture
            elif request.entity_type.entity_type == EntityType.SHELF:
                entity_class = Shelf
            elif request.entity_type.entity_type == EntityType.TABLE:
                entity_class = Table
            else:
                entity_class = None

            if entity_class is None:
                response.result.result_type = Result.ERROR_INVALID_INPUT
                response.result.error = "Unknown entity type"
            else:
                stmt = select(entity_class.id).order_by(
                    entity_class.stamp_sec, entity_class.stamp_nanosec
                )
                result = await session.execute(stmt)
                entities = result.scalars().all()
                response.entities = entities
                if len(entities) > 0:
                    latest = entities[len(entities) - 1]
                    response.stamp = latest
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

    async def update_entity_callback(
        self, request: UpdEntity.Request, response: UpdEntity.Response
    ):
        async with self.Session(response) as session:
            entity = await session.get(Entity, request.entityid)
            if entity is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            entity.description = request.description
            entity.pose = PoseData(request.pose)
            entity.frame_id = request.pose_reference_frame
            entity.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_cupboard_callback(
        self, request: UpdCupboard.Request, response: UpdCupboard.Response
    ):
        async with self.Session(response) as session:
            cupboard = await session.get(Cupboard, request.entityid)
            if cupboard is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            cupboard.description = request.description
            cupboard.pose = PoseData(request.pose)
            cupboard.frame_id = request.pose_reference_frame
            cupboard.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_door_callback(
        self, request: UpdDoor.Request, response: UpdDoor.Response
    ):
        async with self.Session(response) as session:
            door = await session.get(Door, request.entityid)
            if door is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            door.description = request.description
            door.pose = PoseData(request.pose)
            door.frame_id = request.pose_reference_frame
            door.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_furniture_callback(
        self, request: UpdFurniture.Request, response: UpdFurniture.Response
    ):
        async with self.Session(response) as session:
            furniture = await session.get(Furniture, request.entityid)
            if furniture is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            furniture.description = request.description
            furniture.pose = PoseData(request.pose)
            furniture.frame_id = request.pose_reference_frame
            furniture.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_human_callback(
        self, request: UpdHuman.Request, response: UpdHuman.Response
    ):
        async with self.Session(response) as session:
            human = await session.get(Human, request.entityid)
            if human is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            human.description = request.description
            human.pose = PoseData(request.pose)
            human.shape = request.shape
            human.frame_id = request.pose_reference_frame
            human.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_pickable_callback(
        self, request: UpdPickable.Request, response: UpdPickable.Response
    ):
        async with self.Session(response) as session:
            pickable = await session.get(Pickable, request.entityid)
            if pickable is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            pickable.description = request.description
            pickable.pose = PoseData(request.pose)
            pickable.frame_id = request.pose_reference_frame
            pickable.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_shelf_callback(
        self, request: UpdShelf.Request, response: UpdShelf.Response
    ):
        async with self.Session(response) as session:
            shelf = await session.get(Shelf, request.entityid)
            if shelf is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            shelf.description = request.description
            shelf.pose = PoseData(request.pose)
            shelf.frame_id = request.pose_reference_frame
            shelf.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_table_callback(
        self, request: UpdTable.Request, response: UpdTable.Response
    ):
        async with self.Session(response) as session:
            table = await session.get(Table, request.entityid)
            if table is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            table.description = request.description
            table.pose = PoseData(request.pose)
            table.frame_id = request.pose_reference_frame
            table.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_pose_callback(
        self, request: UpdPose.Request, response: UpdPose.Response
    ):
        async with self.Session(response) as session:
            entity = await session.get(Entity, request.entityid)
            if entity is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            entity.pose = PoseData(request.pose)
            entity.frame_id = request.pose_reference_frame
            entity.stamp = TimeData(request.stamp)
            await session.commit()
        return response

    async def update_shape_callback(
        self, request: UpdShape.Request, response: UpdShape.Response
    ):
        async with self.Session(response) as session:
            entity_shape = await session.get(
                Entity, request.entityid, options=(joinedload(Entity.shape),)
            )
            if entity_shape is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            if entity_shape.shape is None:
                entity_shape.shape = Shape(data=request.shape)
            else:
                entity_shape.shape.data = request.shape
            await session.commit()
        return response

    async def del_entities_callback(
        self, request: DelEntities.Request, response: DelEntities.Response
    ):
        async with self.Session(response) as session:
            entity_ids = request.entityids
            if not entity_ids:
                response.result.result_type = Result.ERROR_INVALID_INPUT
                response.result.error = "No entity IDs provided"
                return response
            stmt = delete(Entity).where(Entity.id.in_(entity_ids))
            result = await session.execute(stmt)
            if result.rowcount == 0:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
            return response

    async def furniture_get_pickable_callback(
        self, request: GetReference.Request, response: GetReference.Response
    ):
        async with self.Session(response) as session:
            furniture = await session.execute(
                select(Furniture)
                .where(Furniture.id == request.entityid)
                .options(joinedload(Furniture.pickables))
            )
            furniture = furniture.scalar_one_or_none()
            if furniture is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
        response.entities = list(map(lambda pickable: pickable.id, furniture.pickables))
        return response

    async def pickable_get_furniture_callback(
        self, request: GetReference.Request, response: GetReference.Response
    ):
        async with self.Session(response) as session:
            pickable = await session.execute(
                select(Pickable)
                .where(Pickable.id == request.entityid)
                .options(joinedload(Pickable.located_on_id))
            )
            pickable = pickable.scalar_one_or_none()
            if pickable is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
        response.entities = pickable.located_on_id
        return response

    async def cupboard_get_shelf_callback(
        self, request: GetReference.Request, response: GetReference.Response
    ):
        async with self.Session(response) as session:
            cupboard = await session.execute(
                select(Cupboard)
                .where(Cupboard.id == request.entityid)
                .options(joinedload(Cupboard.shelves))
            )
            cupboard = cupboard.scalar_one_or_none()
            if cupboard is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
        response.entities = list(map(lambda shelf: shelf.id, cupboard.shelves))
        return response

    async def shelf_get_cupboard_callback(
        self, request: GetReference.Request, response: GetReference.Response
    ):
        async with self.Session(response) as session:
            shelf = await session.execute(
                select(Shelf)
                .where(Shelf.id == request.entityid)
                .options(joinedload(Shelf.cupboard))
            )
            shelf = shelf.scalar_one_or_none()
            if shelf is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
        response.entities = shelf.cupboard_id
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
