# ARLAB decision making

This package contains the global decision maker.
It contains a py_trees-based behavior tree

## Main behavior tree

```mermaid
flowchart TD
    R[Behavior tree root] --> S0(Sequence memory=false)
    S0 --> CS[CheckSafety]
    S0 --> S1(Selector)
    S1 --> T1(Task1: Help me carry)
    S1 --> T2(Task2: Storing groceries)
```

## Subtree Task1: Help me carry

```mermaid
flowchart TD
    R[Task1: Help me carry] --> S0(Sequence memory=false)
```

## Subtree Task2

### Subtree Task2: Storing groceries

```mermaid
flowchart TD
    StoringGroceries[Task2 
    Storing groceries] --> S0(Sequence memory=false)
    S0 --> CS[WaitForTaskStart]
    S0 --> S1(Selector)
    S1 --> T1[Done]
    S1 --> S2(Sequence
    memory = false)
    S1 --> SFF[SearchForFurniture]
    S2 --> CF[CupboardFound]
    S2 --> TF[TableFound]
    S2 --> SG[StoreGrocery]
```

### StoreGrocery

```mermaid
flowchart TD
    StoreGrocery(Sequence
     memory = true) --> MT(MoveToTable)
     StoreGrocery --> CFI(CheckForItems)
     StoreGrocery --> PI(PickItem)
     StoreGrocery --> MTC(MoveToCupboard)
     StoreGrocery --> CFTP(CheckForTargetPlacement)
     StoreGrocery --> PlI(PlaceItem)
```
