# ROS 2 Architecture Analysis

This directory contains the reproducible static analysis of ROS 2
communication interfaces in the ARLAB project.

The analysis combines manually documented communication relationships
with automatically extracted interface declarations.

## Maintainer

Jessica Herrmann (jessica.herrmann@uni-a.de)

The architecture analysis is maintained as a repository-level tooling
workflow. It is not a separate ROS 2 package.

## Running the analysis

Run the following command from the repository root:

    python3 scripts/architecture/run_architecture_analysis.py

The pipeline performs nine steps:

1. Extract Python ROS 2 interface declarations.
2. Extract C++ ROS 2 interface declarations.
3. Combine Python and C++ interface declarations.
4. Detect static communication candidates.
5. Merge automatic candidates with manual architecture documentation.
6. Analyze unmatched and open communication endpoints.
7. Generate the merged architecture diagram.
8. Generate the endpoint detail diagram.
9. Generate the endpoint summary diagram.

The pipeline stops if a step fails. Graphviz is required for diagram
generation.

## Directory structure

- ros2_interfaces.csv
  Manually maintained architecture relationships.

- ros2_connection_exclusions.csv
  Reviewed connections excluded from the operational architecture,
  including their documented reasons.

- generated/interfaces/
  Extracted declarations, automatic connections and unmatched interfaces.

- generated/validation/
  Merged architecture, review entries and analysis reports.

- generated/diagrams/
  Architecture diagrams in DOT and SVG formats.

## Data sources

### Manual documentation

The file ros2_interfaces.csv contains manually curated communication
relationships. It is not overwritten by the analysis pipeline.

### Reviewed exclusions

The file ros2_connection_exclusions.csv contains explicitly reviewed
connections that are excluded from the operational architecture.

Exclusion rules identify connections by source component, target
component, interface kind, interface name and interface type.

The automatic analysis continues to detect these connections.
During merging, matching automatic-only candidates receive the
merge status `excluded`.

Excluded connections remain in the merged CSV for traceability,
but are omitted from the architecture diagram and do not generate
open review entries.

An exclusion does not mean that the communication interface is
technically absent. It documents a decision about its relevance
to the operational architecture.

### Static interface extraction

The file generated/interfaces/ros2_interfaces_v3.csv contains
statically extracted ROS 2 interface declarations.

The extractor attempts to resolve literal names, static expressions,
constructor defaults and parameter defaults.

Unresolved declarations retain their resolution status.

### Automatic connection analysis

The file generated/interfaces/ros2_connections_auto.csv contains
automatically identified communication candidates.

The file generated/interfaces/ros2_unmatched_interfaces.csv contains
interface declarations without a matched endpoint.

Unmatched interfaces do not necessarily indicate software defects.

## Extended analysis outputs

The pipeline generates additional artifacts for reviewing the
architecture and its unresolved communication endpoints.

### Combined Python and C++ interfaces

- `generated/interfaces/ros2_interfaces_cpp.csv`
  Statically extracted C++ ROS 2 interface declarations.

- `generated/interfaces/ros2_interfaces_combined.csv`
  Combined Python and C++ interface declarations.

### Additional communication candidates

- `generated/interfaces/ros2_connections_namespace_candidates.csv`
  Potential connections whose compatibility depends on ROS 2
  namespace resolution.

- `generated/interfaces/ros2_connections_launch_candidates.csv`
  Potential connections supported by inspected launch configuration.

These candidates are not equivalent to verified runtime connections.

### Endpoint analysis

- `generated/interfaces/ros2_endpoint_analysis.csv`
  Classification of interface endpoints, including declarations
  without a statically detected counterpart.

- `generated/interfaces/ros2_endpoint_analysis_report.md`
  Summary of the endpoint analysis.

### Additional diagrams

- `generated/diagrams/ros2_open_endpoints.dot`
- `generated/diagrams/ros2_open_endpoints.svg`

  Detailed visualization of analyzed communication endpoints.

- `generated/diagrams/ros2_endpoint_summary.dot`
- `generated/diagrams/ros2_endpoint_summary.svg`

  Summary visualization of endpoints by package.

Open endpoints indicate that no corresponding endpoint was detected
by the static analysis. They do not prove that runtime communication
is impossible.

## Evidence categories

The merged architecture distinguishes three categories:

- [S] Static match:
  The relationship appears in both the manual documentation and
  the automatic analysis.

- [M] Manual only:
  The relationship is documented manually but was not confirmed
  by the automatic analysis.

- [A] Automatic only:
  The relationship was identified automatically but is not
  included in the manual documentation.

These categories describe agreement between manual documentation
and static analysis. They do not prove runtime communication.

The additional merge status `excluded` identifies automatically
detected connections that have been explicitly excluded from the
operational architecture. Excluded connections are retained in
the merged CSV but are not represented in the diagram.

## Additional diagram annotations

The merged architecture diagram uses additional annotations:

- `[N]` Namespace candidate:
  A potential connection whose compatibility depends on ROS 2
  namespace resolution. Static analysis alone does not establish
  that the endpoints communicate at runtime.

- `[L]` Launch candidate:
  A potential connection supported by inspected launch
  configuration. The candidate is not a confirmed static match
  or verified runtime connection.

- `[O]` Open endpoint:
  A publisher for which the analysis did not identify a matching
  subscriber. Open endpoints are shown as annotations rather
  than confirmed communication relationships.

The diagram distinguishes communication relationships from open
endpoint annotations. An annotation does not introduce a
corresponding subscriber or additional communication relationship.

### Knowledge visualization and RViz

The knowledge visualization publisher uses the topic
`/arlab/knowledge/visualization` with the interface type
`visualization_msgs/msg/MarkerArray`.

The relationship to RViz is retained from the manual architecture
documentation. However, a corresponding RViz subscriber was not
identified in the inspected RViz configuration.

The diagram therefore marks this relationship as not statically
verified. Runtime communication remains unverified.

## Reviewing results

After running the pipeline:

1. Review generated/validation/ros2_architecture_merge_report.md.
2. Examine generated/validation/ros2_architecture_merge_review.csv.
3. Inspect generated/interfaces/ros2_endpoint_analysis_report.md.
4. Check unresolved and unmatched interfaces where relevant.
5. Verify candidate relationships before updating the manual CSV.
6. Rerun the pipeline after making changes.

Automatic-only relationships require review before acceptance.

Reviewed connections that should not appear in the operational
architecture can be documented in ros2_connection_exclusions.csv.
Exclusion decisions should include a reason and be reconsidered
if the corresponding component becomes part of the operational system.

## Architecture diagram

The diagram generator produces two persistent output files:

- `generated/diagrams/ros2_architecture_complete.svg`
  The final architecture diagram, combining the component
  communication graph, interface legend and evidence categories.

- `generated/diagrams/ros2_architecture_complete.dot`
  The Graphviz source of the architecture graph, excluding the
  separately rendered legend.

The graph and legend are rendered separately and combined into
the final SVG. Intermediate DOT and SVG files are created in a
temporary directory and automatically removed after generation.

## Limitations

This is a static source-code analysis, not a runtime communication trace.

Important limitations include:

- Runtime communication is not verified.
- Dynamically constructed interface names may remain unresolved.
- C++ interface declarations may not be comprehensively covered.
- Matching interface names and types does not guarantee communication.
- Manual-only relationships require independent verification.
- Development and demonstration components may appear in the results.

The diagram is an architecture documentation and review artifact,
not a complete representation of observed runtime communication.
