import rclpy
import rclpy.clock
from arlab_knowledge_interfaces.msg import EntityType
from arlab_knowledge_interfaces.srv import (
    AddCupboard,
    AddDoor,
    AddEntity,
    AddFurniture,
    AddHuman,
    AddMap,
    AddPickable,
    AddShelf,
    AddTable,
    GetDescription,
    GetEntities,
    GetMap,
    GetPose,
    GetReference,
    GetShape,
    UpdCupboard,
    UpdDoor,
    UpdEntity,
    UpdFurniture,
    UpdHuman,
    UpdPickable,
    UpdPose,
    UpdReference,
    UpdShape,
    UpdShelf,
    UpdTable,
)
from geometry_msgs.msg import Point, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node


def create_pose():
    return Pose(
        position=Point(x=1.0, y=2.0, z=0.5),
        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
    )


class DatabaseServiceTester(Node):
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
            self.test_get_reference,
            self.test_get_shape,
            self.test_get_map,
            self.test_update_entity,
            self.test_update_cupboard,
            self.test_update_door,
            self.test_update_furniture,
            self.test_update_human,
            self.test_update_pickable,
            self.test_update_pose,
            self.test_update_reference,
            self.test_update_shape,
            self.test_update_shelf,
            self.test_update_table,
            self.test_update_entity,
            self.test_update_cupboard,
            self.test_update_door,
            self.test_update_furniture,
            self.test_update_human,
            self.test_update_pickable,
            self.test_update_pose,
            self.test_update_reference,
            self.test_update_shape,
            self.test_update_shelf,
            self.test_update_table,
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

        # 🟡 Replace these with actual IDs from your database
        self.entity_id = 1
        self.parent_id = 1
        self.child_id = 2

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

    async def test_add_entity(self):
        request = AddEntity.Request()
        request.description = "TestEntity"
        request.pose = create_pose()
        request.pose_reference_frame = "map"
        request.stamp = self.create_stamp()

        response = await self.call_service(
            AddEntity, f"{self.prefix}/add_entity", request
        )
        self.entity_id = response.entityid
        self.get_logger().info(
            f"AddEntity response: {response.entityid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_add_furniture(self):
        request = AddFurniture.Request()
        request.description = "TestFurniture"
        request.pose = create_pose()
        request.pose_reference_frame = "map"
        request.stamp = self.create_stamp()

        response = await self.call_service(
            AddFurniture, f"{self.prefix}/add_furniture", request
        )
        self.furniture_id = response.entityid
        self.get_logger().info(
            f"AddFurniture response: {response.entityid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_add_human(self):
        request = AddHuman.Request()
        request.description = "TestHuman"
        request.pose = create_pose()
        request.pose_reference_frame = "map"
        request.stamp = self.create_stamp()

        response = await self.call_service(
            AddHuman, f"{self.prefix}/add_human", request
        )
        self.human_id = response.entityid
        self.get_logger().info(
            f"AddHuman response: {response.entityid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_add_door(self):
        request = AddDoor.Request()
        request.description = "TestDoor"
        request.pose = create_pose()
        request.pose_reference_frame = "map"
        request.width = 0.9
        request.open = "true"
        request.stamp = self.create_stamp()

        response = await self.call_service(AddDoor, f"{self.prefix}/add_door", request)
        self.door_id = response.entityid
        self.get_logger().info(
            f"AddDoor response: {response.entityid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_add_cupboard(self):
        request = AddCupboard.Request()
        request.description = "TestCupboard"
        request.pose = create_pose()
        request.pose_reference_frame = "map"
        request.height = 1.8
        request.width = 0.8
        request.open = "false"
        request.stamp = self.create_stamp()

        response = await self.call_service(
            AddCupboard, f"{self.prefix}/add_cupboard", request
        )
        self.cupboard_id = response.entityid
        self.get_logger().info(
            f"AddCupboard response: {response.entityid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_add_shelf(self):
        request = AddShelf.Request()
        request.cupboard_id = self.cupboard_id
        request.description = "TestShelf"
        request.pose = create_pose()
        request.pose_reference_frame = "map"
        request.width = 0.5
        request.height = 0.7
        request.stamp = self.create_stamp()

        response = await self.call_service(
            AddShelf, f"{self.prefix}/add_shelf", request
        )
        self.shelf_id = response.entityid
        self.get_logger().info(
            f"AddShelf response: {response.entityid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_add_table(self):
        request = AddTable.Request()
        request.description = "TestTable"
        request.pose = create_pose()
        request.pose_reference_frame = "map"
        request.height = 0.75
        request.stamp = self.create_stamp()

        response = await self.call_service(
            AddTable, f"{self.prefix}/add_table", request
        )
        self.table_id = response.entityid
        self.get_logger().info(
            f"AddTable response: {response.entityid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_add_pickable(self):
        request = AddPickable.Request()
        request.description = "TestPickable"
        request.pose = create_pose()
        request.pose_reference_frame = "map"
        request.max_picking_force = 5.0
        request.stamp = self.create_stamp()

        response = await self.call_service(
            AddPickable, f"{self.prefix}/add_pickable", request
        )
        self.pickable_id = response.entityid
        self.get_logger().info(
            f"AddPickable response: {response.entityid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_add_map(self):
        request = AddMap.Request()
        request.grid = OccupancyGrid()
        request.grid.header.stamp = self.create_stamp()

        response = await self.call_service(AddMap, f"{self.prefix}/add_map", request)
        self.pickable_id = response.mapid
        self.get_logger().info(
            f"AddMap response: {response.mapid} ({response.result.result_type}: "
            f"{response.result.error})"
        )

    async def test_get_description(self):
        req = GetDescription.Request()
        req.entityid = self.entity_id
        res = await self.call_service(
            GetDescription, f"{self.prefix}/get_description", req
        )
        self.get_logger().info(
            f"[GetDescription] ID: {req.entityid} → '{res.description}' "
            f"({res.result.result_type}: "
            f"{res.result.error})"
        )

    async def test_get_entities(self):
        req = GetEntities.Request()
        req.entity_type.entity_type = EntityType.ENTITY
        res = await self.call_service(GetEntities, f"{self.prefix}/get_entities", req)
        self.get_logger().info(
            f"[GetEntities] Found: {res.entities} "
            f"({res.result.result_type}: "
            f"{res.result.error})"
        )

    async def test_get_pose(self):
        req = GetPose.Request()
        req.entityid = self.entity_id
        res = await self.call_service(GetPose, f"{self.prefix}/get_pose", req)
        pos = res.pose.position
        self.get_logger().info(
            f"[GetPose] ID: {req.entityid} → (x={pos.x}, y={pos.y}, z={pos.z}) in "
            f"'{res.frame_id}' ({res.result.result_type}: "
            f"{res.result.error})"
        )

    async def test_get_reference(self):
        req = GetReference.Request()
        req.entityid = self.entity_id
        res = await self.call_service(
            GetReference, f"{self.prefix}/furniture_get_pickable", req
        )
        self.get_logger().info(
            f"[GetReference] ID: {req.entityid} → refs: {res.entities} "
            f"({res.result.result_type}: "
            f"{res.result.error})"
        )

    async def test_get_shape(self):
        req = GetShape.Request()
        req.entityid = self.entity_id
        res = await self.call_service(GetShape, f"{self.prefix}/get_shape", req)
        self.get_logger().info(
            f"[GetShape] ID: {req.entityid} → shape: {res.shape} "
            f"({res.result.result_type}: "
            f"{res.result.error})"
        )

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
            f"[GetMap] → has_data: {has_data} ({res.result.result_type}: "
            f"{res.result.error})"
        )

    async def test_update_entity(self):
        req = UpdEntity.Request()
        req.entityid = self.entity_id
        req.description = "Updated Entity"
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdEntity, f"{self.prefix}/upd_entity", req)
        self.get_logger().info(f"[UpdEntity] → {res.result.result_type}")

    async def test_update_cupboard(self):
        req = UpdCupboard.Request()
        req.entityid = self.entity_id
        req.description = "Updated Cupboard"
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.height = 2.0
        req.width = 0.7
        req.open = "true"
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdCupboard, f"{self.prefix}/upd_cupboard", req)
        self.get_logger().info(f"[UpdCupboard] → {res.result.result_type}")

    async def test_update_door(self):
        req = UpdDoor.Request()
        req.entityid = self.entity_id
        req.description = "Updated Door"
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdDoor, f"{self.prefix}/upd_door", req)
        self.get_logger().info(f"[UpdDoor] → {res.result.result_type}")

    async def test_update_furniture(self):
        req = UpdFurniture.Request()
        req.entityid = self.entity_id
        req.description = "Updated Furniture"
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdFurniture, f"{self.prefix}/upd_furniture", req)
        self.get_logger().info(f"[UpdFurniture] → {res.result.result_type}")

    async def test_update_human(self):
        req = UpdHuman.Request()
        req.entityid = self.entity_id
        req.description = "Updated Human"
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.shape = "tall"
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdHuman, f"{self.prefix}/upd_human", req)
        self.get_logger().info(f"[UpdHuman] → {res.result.result_type}")

    async def test_update_pickable(self):
        req = UpdPickable.Request()
        req.entityid = self.entity_id
        req.description = "Updated Pickable"
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.max_picking_force = 10.0
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdPickable, f"{self.prefix}/upd_pickable", req)
        self.get_logger().info(f"[UpdPickable] → {res.result.result_type}")

    async def test_update_pose(self):
        req = UpdPose.Request()
        req.entityid = self.entity_id
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdPose, f"{self.prefix}/upd_pose", req)
        self.get_logger().info(f"[UpdPose] → {res.result.result_type}")

    async def test_update_reference(self):
        req = UpdReference.Request()
        req.parentid = self.parent_id
        req.childid = self.child_id
        req.delete_ref = False  # set to True to test deletion
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdReference, f"{self.prefix}/del_reference", req)
        self.get_logger().info(f"[UpdReference] → {res.result.result_type}")

    async def test_update_shape(self):
        req = UpdShape.Request()
        req.entityid = self.entity_id
        req.shape = "box_1x1x1"
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdShape, f"{self.prefix}/upd_shape", req)
        self.get_logger().info(f"[UpdShape] → {res.result.result_type}")

    async def test_update_shelf(self):
        req = UpdShelf.Request()
        req.entityid = self.entity_id
        req.description = "Updated Shelf"
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.height = 1.2
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdShelf, f"{self.prefix}/upd_shelf", req)
        self.get_logger().info(f"[UpdShelf] → {res.result.result_type}")

    async def test_update_table(self):
        req = UpdTable.Request()
        req.entityid = self.entity_id
        req.description = "Updated Table"
        req.pose = create_pose()
        req.pose_reference_frame = "map"
        req.height = 0.75
        req.stamp = self.create_stamp()
        res = await self.call_service(UpdTable, f"{self.prefix}/upd_table", req)
        self.get_logger().info(f"[UpdTable] → {res.result.result_type}")

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
