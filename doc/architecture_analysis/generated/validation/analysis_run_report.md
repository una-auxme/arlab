# ROS 2 Architecture Analysis Run

- Timestamp (UTC): `2026-11-03T18:43:56+00:00`
- Repository: `/home/jessa/robotics/arlab_docker/src/arlab`
- Python: `3.14.4`
- Duration: `0.38s`

## Results

- Interface declarations: **101**
- Static connection candidates: **23**
- Unmatched declarations: **62**
- Merged architecture relationships: **28**
- Merge review entries: **0**

## Pipeline

- Extract ROS2 interfaces: 0.14s
- Extract C++ ROS2 interfaces: 0.09s
- Combine Python and C++ interfaces: 0.01s
- Match communication endpoints: 0.02s
- Merge architecture documentation: 0.02s
- Generate complete architecture diagram: 0.09s

## Limitations

- Static source analysis does not verify runtime communication.
- C++ parsing may be incomplete in files with syntax errors; external packages may not be represented.
- Configured or conditional interfaces may differ at runtime.
- Manual architecture entries are preserved, but are not automatically verified against current code.
- Component mappings are partly manually maintained.
