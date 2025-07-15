"""Contains the DatabaseServiceTester node for testing the database services.

These tests are not compatible with `pytest`/`colcon test` and need to be manually
executed via `python3 test_database_node.py`.

Maintainers:
    Peter Viechter <peter.viechter@uni-augsburg.de>
    Daniel Gabler <daniel.gabler@uni-augsburg.de>
"""

import arlab_knowledge.db.entities as entities
import rclpy
from arlab_knowledge import test_utils as utils
from arlab_knowledge_interfaces.msg import EntityPickable, EntityType, StatusType
from arlab_knowledge_interfaces.srv import (
    AddEntity,
    AddMap,
    AddStatusEvent,
    DelEntities,
    GetDescription,
    GetEntities,
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
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node


class DatabaseServiceTester(Node):
    """Sequentially executes tests on the database services

    All tests are specified in self.test_services
    """

    def __init__(self):
        super().__init__("database_service_tester")
        self.prefix = "/arlab/knowledge"
        self.service_client_group = MutuallyExclusiveCallbackGroup()

        self.test_services = [
            self.test_add_entity,
            self.test_add_furniture,
            self.test_add_human,
            self.test_add_door,
            self.test_add_cupboard,
            self.test_add_shelf,
            self.test_add_table,
            self.test_add_pickable,
            self.test_add_map,
            self.test_get_description,
            self.test_get_entities,
            self.test_get_pose,
            self.test_furniture_update_pickable,
            self.test_furniture_get_pickable,
            self.test_update_shape,
            self.test_get_shape,
            self.test_get_map,
            self.test_update_entity,
            self.test_update_cupboard,
            self.test_update_door,
            self.test_update_furniture,
            self.test_update_human,
            self.test_update_pickable,
            self.test_update_pose,
            self.test_update_shelf,
            self.test_update_table,
            # self.test_del_entities,
            self.test_add_status_event,
            self.test_get_status_events,
        ]

        self.entity_id = 0
        self.cupboard_id = 0
        self.door_id = 0
        self.furniture_id = 0
        self.human_id = 0
        self.map_id = 0
        self.pickable_id = 0
        self.shelf_id = 0
        self.table_id = 0

        self.current_test = 0
        self.timer = self.create_timer(0.01, self.run_next_test)

    def create_stamp(self):
        return self.get_clock().now().to_msg()

    async def call_service(self, srv_type, service_name, request):
        client = self.create_client(
            srv_type, service_name, callback_group=self.service_client_group
        )
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {service_name}...")
        future = client.call_async(request)
        await future
        return future.result()

    def log_result(self, name, result):
        if result.result_type == result.SUCCESS:
            self.get_logger().info(f"[{name}] ✅ SUCCESS")
        else:
            self.get_logger().error(
                f"[{name}] ❌ ERROR {result.result_type}: {result.error}"
            )

    async def generic_test_add_entity(self, gen_fn):
        request = AddEntity.Request()
        request.data = gen_fn().to_ros_msg()
        # For shelves:
        request.data.furniture.shelf.cupboard_id = self.cupboard_id

        response = await self.call_service(
            AddEntity, f"{self.prefix}/add_entity", request
        )
        class_name = entities.entity_msg_type_to_class(
            request.data.entity_type
        ).__name__
        self.log_result(
            f"AddEntity: {class_name}",
            response.result,
        )
        return response.entityid

    async def test_add_entity(self):
        self.entity_id = await self.generic_test_add_entity(utils.get_entity)

    async def test_add_furniture(self):
        self.furniture_id = await self.generic_test_add_entity(utils.get_furniture)

    async def test_add_human(self):
        self.human_id = await self.generic_test_add_entity(utils.get_human)

    async def test_add_door(self):
        self.door_id = await self.generic_test_add_entity(utils.get_door)

    async def test_add_cupboard(self):
        self.cupboard_id = await self.generic_test_add_entity(utils.get_cupboard)

    async def test_add_shelf(self):
        self.shelf_id = await self.generic_test_add_entity(utils.get_shelf)

    async def test_add_table(self):
        self.table_id = await self.generic_test_add_entity(utils.get_table)

    async def test_add_pickable(self):
        self.pickable_id = await self.generic_test_add_entity(utils.get_pickable)

    async def test_add_map(self):
        request = AddMap.Request()
        request.grid = OccupancyGrid()
        request.grid.header.stamp = self.create_stamp()

        response = await self.call_service(AddMap, f"{self.prefix}/add_map", request)
        self.map_id = response.mapid
        self.log_result("AddMap", response.result)

    async def test_get_description(self):
        req = GetDescription.Request()
        req.entityid = self.entity_id
        res = await self.call_service(
            GetDescription, f"{self.prefix}/get_description", req
        )
        self.get_logger().info(
            f"[GetDescription] ID: {req.entityid} → '{res.description}'"
        )
        self.log_result("GetDescription", res.result)

    async def test_get_entities(self):
        req = GetEntities.Request()
        req.entity_type.id = EntityType.ENTITY
        res = await self.call_service(GetEntities, f"{self.prefix}/get_entities", req)
        self.get_logger().info(f"[GetEntities] Found: {res.entities}")
        self.log_result("GetEntities", res.result)

    async def test_get_pose(self):
        req = GetPose.Request()
        req.entityid = self.entity_id
        res = await self.call_service(GetPose, f"{self.prefix}/get_pose", req)
        pos = res.pose.position
        self.get_logger().info(
            f"[GetPose] ID: {req.entityid} → (x={pos.x}, y={pos.y}, z={pos.z})"
        )
        self.log_result("GetPose", res.result)

    async def test_furniture_get_pickable(self):
        req = GetReference.Request()
        req.entityid = self.furniture_id
        res = await self.call_service(
            GetReference, f"{self.prefix}/furniture_get_pickable", req
        )
        self.get_logger().info(
            f"[GetReference] ID: {req.entityid} → refs: {res.entities}"
        )
        self.log_result("GetReference", res.result)

    async def test_get_shape(self):
        req = GetShape.Request()
        req.entityid = self.entity_id
        res = await self.call_service(GetShape, f"{self.prefix}/get_shape", req)
        self.get_logger().info(f"[GetShape] ID: {req.entityid} → shape: {res.shape}")
        self.log_result("GetShape", res.result)

    async def test_get_map(self):
        req = GetMap.Request()
        now = self.create_stamp()
        max_age_stamp = self.create_stamp()
        max_age_stamp.sec -= 10
        req.min_age_stamp = now
        req.max_age_stamp = max_age_stamp
        req.backwards_index = 0
        res = await self.call_service(GetMap, f"{self.prefix}/get_map", req)
        has_data = bool(res.grid.data)
        self.get_logger().info(
            f"[GetMap] → has_data: {has_data} ({res.result.result_type}"
        )
        self.log_result("GetMap", res.result)

    async def generic_test_update_entity(self, req: UpdEntity.Request):
        req.data.stamp = self.create_stamp()
        res = await self.call_service(UpdEntity, f"{self.prefix}/upd_entity", req)
        class_name = entities.entity_msg_type_to_class(req.data.entity_type).__name__
        self.log_result(
            f"UpdEntity: {class_name}",
            res.result,
        )

    async def test_update_entity(self):
        req = UpdEntity.Request()
        req.entityid = self.entity_id
        req.data.description = "Updated Entity"
        req.data.pose = utils.create_pose2()
        req.data.pose_reference_frame = "map"
        await self.generic_test_update_entity(req)

    async def test_update_cupboard(self):
        req = UpdEntity.Request()
        req.entityid = self.cupboard_id
        req.data.entity_type.id = EntityType.CUPBOARD
        req.data.description = "Updated Cupboard"
        req.data.pose = utils.create_pose2()
        req.data.pose_reference_frame = "map"
        req.data.furniture.cupboard.height = 2.0
        req.data.furniture.cupboard.width = 0.7
        req.data.furniture.cupboard.open = "true"
        req.data.stamp = self.create_stamp()
        await self.generic_test_update_entity(req)

    async def test_update_door(self):
        req = UpdEntity.Request()
        req.entityid = self.door_id
        req.data.entity_type.id = EntityType.DOOR
        req.data.description = "Updated Door"
        req.data.pose = utils.create_pose2()
        req.data.pose_reference_frame = "map"
        req.data.stamp = self.create_stamp()
        await self.generic_test_update_entity(req)

    async def test_update_furniture(self):
        req = UpdEntity.Request()
        req.entityid = self.furniture_id
        req.data.entity_type.id = EntityType.FURNITURE
        req.data.description = "Updated Furniture"
        req.data.pose = utils.create_pose2()
        req.data.pose_reference_frame = "map"
        req.data.stamp = self.create_stamp()
        await self.generic_test_update_entity(req)

    async def test_update_human(self):
        req = UpdEntity.Request()
        req.entityid = self.human_id
        req.data.entity_type.id = EntityType.HUMAN
        req.data.description = "Updated Human"
        req.data.pose = utils.create_pose2()
        req.data.pose_reference_frame = "map"
        req.data.stamp = self.create_stamp()
        await self.generic_test_update_entity(req)

    async def test_update_pickable(self):
        req = UpdEntity.Request()
        req.entityid = self.pickable_id
        req.data.entity_type.id = EntityType.PICKABLE
        req.data.description = "Updated Pickable"
        req.data.pose = utils.create_pose2()
        req.data.pose_reference_frame = "map"
        req.data.pickable.picking_tag = EntityPickable.TAG_MILK
        req.data.stamp = self.create_stamp()
        await self.generic_test_update_entity(req)

    async def test_update_shelf(self):
        req = UpdEntity.Request()
        req.entityid = self.shelf_id
        req.data.entity_type.id = EntityType.SHELF
        req.data.furniture.shelf.cupboard_id = self.cupboard_id
        req.data.description = "Updated Shelf"
        req.data.pose = utils.create_pose2()
        req.data.pose_reference_frame = "map"
        req.data.furniture.shelf.height = 1.2
        req.data.stamp = self.create_stamp()
        await self.generic_test_update_entity(req)

    async def test_update_table(self):
        req = UpdEntity.Request()
        req.entityid = self.table_id
        req.data.entity_type.id = EntityType.TABLE
        req.data.description = "Updated Table"
        req.data.pose = utils.create_pose2()
        req.data.pose_reference_frame = "map"
        req.data.furniture.table.height = 0.75
        req.data.stamp = self.create_stamp()
        await self.generic_test_update_entity(req)

    async def test_update_pose(self):
        req = UpdPose.Request()
        req.entityid = self.entity_id
        req.pose = utils.create_pose2()
        req.pose_reference_frame = "map"
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdPose, f"{self.prefix}/upd_pose", req)
        self.log_result("UpdPose", res.result)

    async def test_furniture_update_pickable(self):
        req = UpdReference.Request()
        req.parentid = self.table_id
        req.childid = self.pickable_id
        req.delete_ref = False  # set to True to test deletion
        req.stamp = self.create_stamp()
        res = await self.call_service(
            UpdReference, f"{self.prefix}/furniture_update_pickable", req
        )
        self.log_result("UpdReference: furniture_update_pickable", res.result)

    async def test_update_shape(self):
        req = UpdShape.Request()
        req.entityid = self.entity_id
        shape = utils.get_shape()
        req.shape = shape.to_ros_msg()
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdShape, f"{self.prefix}/upd_shape", req)
        self.log_result("UpdShape", res.result)

    async def test_del_entities(self):
        req = DelEntities.Request()
        req.entityids = [
            self.entity_id,
            self.cupboard_id,
            self.door_id,
            self.furniture_id,
            self.human_id,
            self.pickable_id,
            self.shelf_id,
            self.table_id,
        ]
        res = await self.call_service(DelEntities, f"{self.prefix}/del_entities", req)
        self.log_result("DelEntities", res.result)

    async def test_add_status_event(self):
        req = AddStatusEvent.Request()
        req.event.sender = "TestSender"
        req.event.stamp = self.create_stamp()
        req.event.status.is_ok = True
        req.event.status.status_type.id = StatusType.STATUS_SAFETY
        res = await self.call_service(
            AddStatusEvent, f"{self.prefix}/add_status_event", req
        )
        self.log_result("AddStatusEvent", res.result)

    async def test_get_status_events(self):
        req = GetStatusEvents.Request()
        now = self.create_stamp()
        max_age_stamp = self.create_stamp()
        max_age_stamp.sec -= 10
        req.min_age_stamp = now
        req.max_age_stamp = max_age_stamp
        req.status_type.id = StatusType.STATUS_SAFETY
        res = await self.call_service(
            GetStatusEvents, f"{self.prefix}/get_status_events", req
        )
        self.get_logger().info(f"[GetStatusEvents] Found: {res.events}")
        self.log_result("GetStatusEvents", res.result)

    async def run_next_test(self):
        if self.current_test >= len(self.test_services):
            self.get_logger().info("All service tests completed.")
            rclpy.shutdown()
            return
        test_fn = self.test_services[self.current_test]
        await test_fn()
        self.current_test += 1


def main(args=None):
    rclpy.init(args=args)
    tester = DatabaseServiceTester()
    rclpy.spin(tester)


if __name__ == "__main__":
    main()
