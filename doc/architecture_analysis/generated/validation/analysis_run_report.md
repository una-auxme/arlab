# ROS 2 Architecture Analysis Run

- Timestamp (UTC): `2026-11-03T07:30:27+00:00`
- Repository: `/home/jessa/robotics/arlab_docker/src/arlab`
- Python: `3.14.4`
- Duration: `0.28s`

## Results

- Interface declarations: **83**
- Static connection candidates: **19**
- Unmatched declarations: **51**
- Merged architecture relationships: **28**
- Merge review entries: **0**

## Pipeline

- Extract ROS2 interfaces: 0.14s
- Match communication endpoints: 0.03s
- Merge architecture documentation: 0.02s
- Generate complete architecture diagram: 0.09s

## Limitations

- Static source analysis does not verify runtime communication.
- C++ and external packages may not be fully represented.
- Configured or conditional interfaces may differ at runtime.
- Manual architecture entries are preserved, but are not automatically verified against current code.
- Component mappings are partly manually maintained.
