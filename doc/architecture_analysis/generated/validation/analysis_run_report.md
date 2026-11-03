# ROS 2 Architecture Analysis Run

- Timestamp (UTC): `2026-11-03T07:46:01+00:00`
- Repository: `/home/jessa/robotics/arlab_docker/src/arlab`
- Python: `3.14.4`
- Duration: `0.29s`

## Results

- Interface declarations: **78**
- Static connection candidates: **19**
- Unmatched declarations: **46**
- Merged architecture relationships: **28**
- Merge review entries: **0**

## Pipeline

- Extract ROS2 interfaces: 0.17s
- Match communication endpoints: 0.02s
- Merge architecture documentation: 0.02s
- Generate complete architecture diagram: 0.09s

## Limitations

- Static source analysis does not verify runtime communication.
- C++ and external packages may not be fully represented.
- Configured or conditional interfaces may differ at runtime.
- Manual architecture entries are preserved, but are not automatically verified against current code.
- Component mappings are partly manually maintained.
