from typing import Any, Dict

import rclpy.logging
from arlab_knowledge_interfaces import msg
from sqlalchemy import ForeignKey, Integer, String
from sqlalchemy.orm import Mapped, composite, mapped_column, relationship

import arlab_knowledge.db as db

from .base import Base
from .ros_adapters.time import TimeData


class PointCloud(Base):
    __tablename__ = "point_cloud"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
