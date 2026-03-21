# ARLAB knowledge

This package provides a database for handling the robot's surroundings, events and maps.

## Package structure

```txt
arlab_knowledge/
├── arlab_knowledge/                # Main Python package
│   ├── database_node.py            # Database node entry point
│   ├── knowledge_visualization.py  # Visualization tool
│   └── db/                         # Database models and adapters
│       ├── base.py                 # Base entity classes
│       ├── entities/               # Entity definitions (furniture, humans, etc.)
│       ├── map.py                  # Map handling
│       ├── ros_adapters/           # ROS2 adapters for data conversion
│       └── status.py               # Status event handling
├── launch/                         # Launch files
├── tests/                          # Unit tests
└── requirements.txt                # Python dependencies
```

## Key features

- SQLAlchemy-based async database for robot state management
- Entity system for furniture, humans, and pickable objects
- ROS2 adapters for seamless integration with robot systems
- Map and status event handling
