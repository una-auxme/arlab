# ROS 2 Architecture Analysis

This directory contains the reproducible static analysis of ROS 2
communication interfaces in the ARLAB project.

The analysis combines manually documented communication relationships
with automatically extracted interface declarations.

## Running the analysis

Run the following command from the repository root:

    python3 scripts/architecture/analyze_architecture.py

The pipeline performs four steps:

1. Extract ROS 2 interface declarations.
2. Match compatible communication endpoints.
3. Merge automatic candidates with manual documentation.
4. Generate the combined architecture diagram.

Graphviz is required for diagram generation.

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

- archive/
  Historical extraction results, analysis outputs and diagrams.

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

## Reviewing results

After running the pipeline:

1. Inspect generated/validation/analysis_run_report.md.
2. Review generated/validation/ros2_architecture_merge_report.md.
3. Examine generated/validation/ros2_architecture_merge_review.csv.
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

## Historical files

The archive/ directory preserves previous extraction results,
legacy analysis outputs and earlier diagrams.

Archived files are not inputs to the current analysis pipeline.
