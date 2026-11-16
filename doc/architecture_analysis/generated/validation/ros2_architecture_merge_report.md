# ROS 2 Architecture Merge Report

## Input

- Manual connections: 27
- Automatic connection candidates: 23
- Namespace-dependent candidates: 1
- Launch-configured candidates: 2

## Merge Results

- Matched connections: 22
- Manual-only connections: 1
- Open endpoints: 3
- Automatic-only connections: 0
- Namespace-dependent candidates: 1
- Launch-configured candidates: 2
- Excluded connections: 1
- Unresolved component mappings: 0
- Ambiguous matches: 0
- Duplicate static candidates: 0
- Interface type mismatches: 0

- Total merged rows: 30
- Review rows: 3

## Interpretation

- A static match indicates matching declarations in source code, not verified runtime communication.
- Manual integration statuses are preserved.
- Automatic-only connections have integration_status `unverified`.
- Namespace candidates are potential connections, not exact matches.
- Launch candidates are supported by launch configuration but are not runtime verified.
- Manual-only connections may depend on C++, external packages, dynamic names, or analysis gaps.
- Component names inferred from source files are provisional and do not establish ROS node identity.

## Files

- Merged architecture: `ros2_architecture_merged.csv`
- Review queue: `ros2_architecture_merge_review.csv`
