# ARLAB decision making

This package contains the global decision maker.
It contains a py_trees-based behavior tree

## Main behavior tree

```mermaid
flowchart TD
    R[Behavior tree root] --> S0(Sequence memory=false)
    S0 --> CS[CheckSafety]
    S0 --> S1(Selector)
    S1 --> T1[Task1: Help me carry]
    S1 --> T2[Task2: Storing groceries]
```

## Subtree Task1

### Task1: Help me carry

```mermaid
flowchart TD
    S0(Sequence memory=false) --> WFTS[WaitForTaskStart]
    S0 --> S1(Selector)
    S1 --> D[Done]
    S1 --> BBB[BringBackBag]
    S1 --> GB[GetBag]
    S1 --> S2(Sequence memory=false)
    S1 --> SFO[SearchForOperator]
    S2 --> OF[OperatorFound]
    S2 --> FO[FollowOperator]
```

### BringBackBag

```mermaid
flowchart TD
    S0(Sequence memory=false) --> CFBIG[CheckForBagInGripper]
    S0 --> ES0(Error selector)
    ES0 --> MTSP[MoveToStartingPosition]
    ES0 --> EH_MTSP[Error handler: MoveToStartingPosition]
```

## Subtree Task2

### Task2: Storing groceries

```mermaid
flowchart TD
    S0(Sequence memory=false)
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
    memory = true) --> MT[MoveToTable]
    StoreGrocery --> CFI[CheckForItems]
    StoreGrocery --> PI[PickItem]
    StoreGrocery --> MTC[MoveToCupboard]
    StoreGrocery --> CFTP[CheckForTargetPlacement]
    StoreGrocery --> PlI[PlaceItem]
```

## Error handling

Any behavior that might require special error handling can be wrapped with an **ErrorSelector**

THe *Action* in the graph below is the behavior being wrapped.

```mermaid
flowchart TD
    ES0(Error selector) --> A[Action]
    ES0 --> EH1_A[Error handler 1]
    ES0 --> EH2_A[Error handler 2]
    ES0 --> EH3_A[Error handler ...]
```

The **ErrorSelector** is a subclass of Selector with the following modifications:

- It only returns SUCCESS if the first child (Action) returns SUCCESS
- It always has memory -> Persistently stays in a running error handler
- The memory is reset if one of the error handlers returns SUCCESS -> Runs the *Action* again in the next tick
- If all children return FAILURE, it returns FAILURE as usual
