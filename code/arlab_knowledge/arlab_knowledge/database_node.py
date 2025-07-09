import asyncio
import os
from contextlib import asynccontextmanager

import rclpy
import sqlalchemy
from arlab_asyncio_executor.executors import AsyncIOExecutor
from arlab_knowledge_interfaces.msg import Result
from arlab_knowledge_interfaces.srv import (
    AddEntity,
    AddMap,
    AddStatusEvent,
    DelEntities,
    GetDescription,
    GetEntities,
    GetEntity,
    GetMap,
    GetPose,
    GetReference,
    GetShape,
    GetStatusEvents,
    UpdEntity,
    UpdPose,
    UpdReference,
    UpdShape,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sqlalchemy import and_, delete, desc, or_, select
from sqlalchemy.exc import DBAPIError, SQLAlchemyError
from sqlalchemy.ext.asyncio import async_sessionmaker, create_async_engine
from sqlalchemy.orm import joinedload

import arlab_knowledge.db.entities as entities
import arlab_knowledge.db.status as status
from arlab_knowledge.db.base import Base
from arlab_knowledge.db.entities.entity import Entity
from arlab_knowledge.db.entities.furniture import (
    Cupboard,
    Furniture,
    Shelf,
)
from arlab_knowledge.db.entities.pickable import Pickable
from arlab_knowledge.db.entities.shape import Shape
from arlab_knowledge.db.map import Map
from arlab_knowledge.db.ros_adapters.occupancy_grid import OccupancyGridData
from arlab_knowledge.db.ros_adapters.pose import PoseData
from arlab_knowledge.db.ros_adapters.time import TimeData
from arlab_knowledge.db.status import RobotStatusEvent

prefix = "/arlab/knowledge"


class DatabaseNode(Node):
    """This node provides services persistently storing knowledge

    The main supported classes are:
    - Entity: An Entity in the world around the robot
    - Map: A map created by/for the SLAM
    - RobotStatusEvent: Events from the different subsystems

    The custom service definitions can be found in the
    arlab_knowledge_interfaces.srv folder
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

    async def async_init(self):
        async with self.db_engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
        self._init_services()

    def _init_services(self):
        self.create_service(
            GetEntities,
            f"{prefix}/get_entities",
            self.get_entities_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetEntity,
            f"{prefix}/get_entity",
            self.get_entity_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetShape,
            f"{prefix}/get_shape",
            self.get_shape_callback,
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
            GetReference,
            f"{prefix}/furniture_get_pickable",
            self.furniture_get_pickable_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetReference,
            f"{prefix}/pickable_get_furniture",
            self.pickable_get_furniture_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetReference,
            f"{prefix}/cupboard_get_shelf",
            self.cupboard_get_shelf_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetReference,
            f"{prefix}/shelf_get_cupboard",
            self.shelf_get_cupboard_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            AddMap,
            f"{prefix}/add_map",
            self.add_map_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetMap,
            f"{prefix}/get_map",
            self.get_map_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            UpdEntity,
            f"{prefix}/upd_entity",
            self.update_entity_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            UpdPose,
            f"{prefix}/upd_pose",
            self.update_pose_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            UpdShape,
            f"{prefix}/upd_shape",
            self.update_shape_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            DelEntities,
            f"{prefix}/del_entities",
            self.del_entities_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            UpdReference,
            f"{prefix}/furniture_update_pickable",
            self.furniture_update_pickable_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            AddStatusEvent,
            f"{prefix}/add_status_event",
            self.add_status_event_callback,
            callback_group=self.reentrant_callback_group,
        )

        self.create_service(
            GetStatusEvents,
            f"{prefix}/get_status_events",
            self.get_status_events_callback,
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
        if (
            response.result.result_type == Result.ERROR_ID_NOT_FOUND
            and not response.result.error
        ):
            response.result.error = "Id not found"

    async def get_entities_callback(
        self, request: GetEntities.Request, response: GetEntities.Response
    ):
        async with self.Session(response) as session:
            entity_class = entities.entity_msg_type_to_class(request.entity_type)

            if entity_class is None:
                response.result.result_type = Result.ERROR_INVALID_INPUT
                response.result.error = "Unknown entity type"
                return response
            stmt = (
                select(entity_class.id, entity_class.stamp)
                .order_by(desc(entity_class.stamp_nanosec))
                .order_by(desc(entity_class.stamp_sec))
            )
            result = await session.execute(stmt)
            entity_ids = result.columns("id").scalars().all()
            rows = result.all()
            response.entities = entity_ids
            if len(rows) > 0:
                latest = rows[0]
                response.stamp = latest[1].time
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
            await session.commit()
        return response

    async def get_entity_callback(
        self, request: GetEntity.Request, response: GetEntity.Response
    ):
        async with self.Session(response) as session:
            entity = await session.get(Entity, request.entityid)
            if entity is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            response.data = entity.to_ros_msg()
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
                response.shape = shape.data
        return response

    async def get_pose_callback(
        self,
        request: GetPose.Request,
        response: GetPose.Response,
    ):
        async with self.Session(response) as session:
            stmt = select(Entity.pose).where(Entity.id == request.entityid)
            result = await session.execute(stmt)
            pose = result.scalar_one_or_none()
            if pose is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
            else:
                response.pose = pose.pose
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
                response.description = description
        return response

    async def add_entity_callback(
        self, request: AddEntity.Request, response: AddEntity.Response
    ):
        async with self.Session(response) as session:
            entity = Entity.from_ros_msg(request.data)
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
            msg_entity_class = entities.entity_msg_type_to_class(
                request.data.entity_type
            )
            if msg_entity_class is None or not isinstance(entity, msg_entity_class):
                # Type mismatch
                response.result.result_type = Result.ERROR_INVALID_INPUT
                response.result.error = "Existing entity "
                f"of type {type(entity)} is not an instance of {msg_entity_class}"
            entity.apply_ros_msg(request.data)
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
            entity.pose_reference_frame = request.pose_reference_frame
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

    async def furniture_update_pickable_callback(
        self, request: UpdReference.Request, response: UpdReference.Response
    ):
        async with self.Session(response) as session:
            furniture_id = request.parentid
            furniture = await session.execute(
                select(Furniture)
                .where(Furniture.id == furniture_id)
                .options(joinedload(Furniture.pickables))
                .limit(1)
            )
            furniture = furniture.unique().scalar_one_or_none()

            if furniture is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                response.result.error = f"Furniture ID {furniture_id} not found"
                return response

            pickable = await session.get(Pickable, request.childid)
            if pickable is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                response.result.error = f"Pickable ID {request.childid} not found"
                return response

            if request.delete_ref:
                if pickable in furniture.pickables:
                    furniture.pickables.remove(pickable)
            else:
                if pickable not in furniture.pickables:
                    furniture.pickables.append(pickable)

            furniture.stamp = TimeData(request.stamp)
            pickable.stamp = TimeData(request.stamp)
            await session.commit()
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
            furniture = furniture.unique().scalar_one_or_none()
            if furniture is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            response.entities = list(
                map(lambda pickable: pickable.id, furniture.pickables)
            )
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

    async def add_map_callback(
        self, request: AddMap.Request, response: AddMap.Response
    ):
        async with self.Session(response) as session:
            map = Map(grid=OccupancyGridData(request.grid))
            async with session.begin():
                session.add(map)
            response.mapid = map.id
        return response

    async def get_map_callback(
        self, request: GetMap.Request, response: GetMap.Response
    ):
        async with self.Session(response) as session:
            map = await session.execute(
                select(Map)
                .filter(
                    or_(
                        request.min_age_stamp.sec > Map.header_stamp_sec,
                        and_(
                            request.min_age_stamp.sec == Map.header_stamp_sec,
                            request.min_age_stamp.nanosec >= Map.header_stamp_nanosec,
                        ),
                    )
                )
                .filter(
                    or_(
                        request.max_age_stamp.sec < Map.header_stamp_sec,
                        and_(
                            request.max_age_stamp.sec == Map.header_stamp_sec,
                            request.max_age_stamp.nanosec <= Map.header_stamp_nanosec,
                        ),
                    )
                )
                .order_by(desc(Map.header_stamp_nanosec))
                .order_by(desc(Map.header_stamp_sec))
                .offset(request.backwards_index)
                .limit(1)
            )
            map = map.scalar_one_or_none()
            if map is None:
                response.result.result_type = Result.ERROR_ID_NOT_FOUND
                return response
            response.grid = map.grid.grid
        return response

    async def add_status_event_callback(
        self, request: AddStatusEvent.Request, response: AddStatusEvent.Response
    ):
        async with self.Session(response) as session:
            event = RobotStatusEvent.from_ros_msg(request.event)
            async with session.begin():
                session.add(event)
            response.eventid = event.id
        return response

    def destroy_node(self):
        asyncio.run(self.db_engine.dispose())
        super().destroy_node()

    async def get_status_events_callback(
        self, request: GetStatusEvents.Request, response: GetStatusEvents.Response
    ):
        async with self.Session(response) as session:
            status_class = status.status_msg_type_to_class(request.status_type)

            if status_class is None:
                response.result.result_type = Result.ERROR_INVALID_INPUT
                response.result.error = "Unknown status type"
                return response

            events = await session.execute(
                select(RobotStatusEvent)
                .join(status_class)
                .options(joinedload(RobotStatusEvent.status))
                .filter(
                    or_(
                        request.min_age_stamp.sec > RobotStatusEvent.stamp_sec,
                        and_(
                            request.min_age_stamp.sec == RobotStatusEvent.stamp_sec,
                            request.min_age_stamp.nanosec
                            >= RobotStatusEvent.stamp_nanosec,
                        ),
                    )
                )
                .filter(
                    or_(
                        request.max_age_stamp.sec < RobotStatusEvent.stamp_sec,
                        and_(
                            request.max_age_stamp.sec == RobotStatusEvent.stamp_sec,
                            request.max_age_stamp.nanosec
                            <= RobotStatusEvent.stamp_nanosec,
                        ),
                    )
                )
                .order_by(desc(RobotStatusEvent.stamp_nanosec))
                .order_by(desc(RobotStatusEvent.stamp_sec))
            )
            events = events.scalars().unique().all()
            response.events = list(map(lambda e: e.to_ros_msg(), events))
            if len(events) > 0:
                response.stamp = events[0].stamp.time
        return response


def main(args=None):
    rclpy.init(args=args)
    database_node = DatabaseNode()
    executor = AsyncIOExecutor(async_init=database_node.async_init())
    executor.add_node(database_node)

    database_node.get_logger().info("Beginning database_node, shut down with CTRL-C")
    executor.spin()


if __name__ == "__main__":
    main()
