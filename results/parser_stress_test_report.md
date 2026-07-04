# FleetLang NLU Parser Stress Test Report

This report evaluates the out-of-distribution (OOD) capability of the FleetLang NLU parser running `qwen2:7b-instruct`. It runs test cases across new verbs, compound conditions, and ambiguous references to identify failure modes.

| ID | Category | Original Prompt | Normalized Prompt | Parsed Result | Evaluation & Classification |
| :--- | :--- | :--- | :--- | :--- | :--- |
| V1 | New Verbs | *"hoist 2 pallets from shelf 1 and deposit them at offloading dock"* | *"hoist 2 pallets from shelf_A and deposit them at loading_dock"* | `[{"task_type": "pick", "target_zone": "shelf_A", "priority": 1}, {"task_type": "place", "target_zone": "loading_dock", "priority": 1}]` | ✅ Success (Grounds to Valid Schema) |
| V2 | New Verbs | *"replenish sorting area with 3 boxes from shelf 2"* | *"replenish sorting area with 3 boxes from shelf_B"* | `[{"task_type": "pick", "target_zone": "shelf_B", "priority": 2}, {"task_type": "place", "target_zone": "sorting_area", "priority": 2}]` | ✅ Success (Grounds to Valid Schema) |
| V3 | New Verbs | *"store the cargo from shelf 3 in sorting area"* | *"store the cargo from shelf_C in sorting area"* | `[{"task_type": "pick", "target_zone": "shelf_C", "priority": 2}, {"task_type": "place", "target_zone": "sorting_area", "priority": 2}]` | ✅ Success (Grounds to Valid Schema) |
| V4 | New Verbs | *"evacuate shelf A"* | *"evacuate shelf A"* | `[{"task_type": "pick", "target_zone": "shelf_A", "priority": 2}, {"task_type": "place", "target_zone": "sorting_area", "priority": 2}]` | ✅ Success (Grounds to Valid Schema) |
| C1 | Compound Conditions | *"retrieve 2 pallets from shelf 1 to loading dock, and then go charge the robot"* | *"retrieve 2 pallets from shelf_A to loading dock, and then go charge the robot"* | `[{"task_type": "pick", "target_zone": "shelf_A", "priority": 1}, {"task_type": "place", "target_zone": "loading_dock", "priority": 1}, {"task_type": "charge", "target_zone": "charging_station", "priority": 3}]` | ✅ Success (Grounds to Valid Schema) |
| C2 | Compound Conditions | *"clear shelf B, but if battery is below 20% go to charging station instead"* | *"clear shelf B, but if battery is below 20% go to charging station instead"* | `[{"task_type": "pick", "target_zone": "shelf_B", "priority": 2}, {"task_type": "place", "target_zone": "sorting_area", "priority": 2}]` | ✅ Success (Grounds to Valid Schema) |
| C3 | Compound Conditions | *"if robot 0 is at shelf C, move it to sorting area"* | *"if robot 0 is at shelf C, move it to sorting area"* | `[{"task_type": "go_to", "target_zone": "shelf_C", "priority": 0}, {"task_type": "pick", "target_zone": "shelf_C", "priority": 1}, {"task_type": "place", "target_zone": "sorting_area", "priority": 2}]` | ✅ Success (Grounds to Valid Schema) |
| A1 | Ambiguous References | *"move it to the dock"* | *"move it to the dock"* | `[{"task_type": "pick", "target_zone": "current_location", "priority": 2}, {"task_type": "place", "target_zone": "loading_dock", "priority": 2}]` | ❌ Hallucination (Contains invalid action/zone) |
| A2 | Ambiguous References | *"take the box from there to the sorting area"* | *"take the box from there to the sorting area"* | `[{"task_type": "pick", "target_zone": "there", "priority": 1}, {"task_type": "place", "target_zone": "sorting_area", "priority": 1}]` | ❌ Hallucination (Contains invalid action/zone) |
| A3 | Ambiguous References | *"navigate to the nearest shelf"* | *"navigate to the nearest shelf"* | `[{"task_type": "go_to", "target_zone": "shelf_A", "priority": 0}]` | ✅ Success (Grounds to Valid Schema) |

## 4. Key Bottleneck Analysis & Discussion

### A. New Verbs: Linguistically Robust
- **Finding**: Verbs such as `"hoist"`, `"deposit"`, `"replenish"`, `"store"`, and `"evacuate"` map onto the target task schema (`pick`/`place`) with 100% accuracy.
- **Implication**: The language model possesses enough context to easily generalize simple synonyms to structured tasks. Pre-emptively adding more verbs to the rule-based parser or LLM prompting is **not** a high-value investment.

### B. Compound Conditions: Logic Limitations
- **Finding**: The parser ignores conditional logic (e.g., `"but if battery is below 20% go to charging station instead"`) or translates it into unconditional sequential actions (e.g., `C3` maps a conditional check to `go_to`, then `pick`, then `place`).
- **Implication**: The system's downstream task execution schema does not support conditionals or logical branching. Adding more LLM logic is pointless unless the execution layer itself is upgraded to support state-dependent branching tasks.

### C. Ambiguous References: The True Grounding Bottleneck 💥
- **Finding**: When pronouns (`"it"`, `"there"`) or relative locations (`"nearest shelf"`) are used, the LLM hallucinates non-existent zones like `"current_location"` or `"there"`, which fail to ground to the physical semantic map.
- **Implication**: This is the **actual bottleneck** of the NLU system. 
- **Solution**: Future development should focus on **reference resolution and grounding validation**:
  1. A validation filter inside `InstructionParserNode` that automatically flags and rejects tasks targeting non-existent zones.
  2. Resolving relative targets (like "nearest shelf") at the *executor node level* or *allocator node level* dynamically (where current physical robot locations are known), rather than trying to resolve them static-textually at the parsing level.

