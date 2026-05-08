# Interactive Single-Arm Scene: Full Implementation and Inspector Checklist

This guide is for building a complete interactive scene from an empty Unity scene to a fully working result, with both zones draggable and mutually constrained.

It reflects the current scripts in this branch:
- `Assets/Scripts/Interactivity/RobotAgentDupe.cs`
- `Assets/Scripts/Interactivity/ModelLoader.cs`
- `Assets/Scripts/Interactivity/BoxDrag.cs`
- `Assets/Scripts/Interactivity/PlacementConstraints.cs`
- `Assets/Scripts/Interactivity/ForceVisuals.cs`
- `Assets/Scripts/Interactivity/StartButton.cs`

---

## 0) Preflight

- [ ] Open project: `Automatic-Unity-Robot-Analysis-box-interactivity`
- [ ] Confirm no compile errors in Console
- [ ] Confirm Sentis and ML-Agents packages are installed (already expected in this repo)
- [ ] Confirm ONNX model exists inside Assets:
  - Recommended: `Assets/Scripts/RobotArm.onnx`

---

## 1) Create a new scene from scratch

- [ ] `File > New Scene`
- [ ] Save as: `Assets/Scenes/InteractiveArm.unity`

Scene root should contain:
- [ ] Main Camera
- [ ] Directional Light

Add required runtime input plumbing:
- [ ] Add `EventSystem` (`GameObject > UI > Event System`)
- [ ] Select `Main Camera` and add `Physics Raycaster`

---

## 2) Add base environment prefab (fastest and safest)

- [ ] Drag `Assets/Prefabs/TrainingArea_0_precision.prefab` into scene
- [ ] Rename instance to `InteractiveArea_0` (optional)

Why this prefab: it already contains arm hierarchy, floor, box, zone meshes, articulations, and most references.

---

## 3) Inspector checklist: RobotArm object

Select child object `RobotArm` under the prefab instance.

### Required components on RobotArm

- [ ] `RobotAgentDupe`
- [ ] `Behavior Parameters` (can stay for compatibility)
- [ ] `Decision Requester` (can stay)
- [ ] `ModelLoader`
- [ ] `ForceVisuals`
- [ ] `DataCollector` (optional but recommended)

### 3.1 RobotAgentDupe fields

Set and verify:

Robot Joint Components
- [ ] `rootBody` -> `Body` (ArticulationBody)
- [ ] `baseRotation` -> `BaseRotation` (ArticulationBody)
- [ ] `shoulderJoint` -> `ShoulderJoint` (ArticulationBody)
- [ ] `elbowJoint` -> `ElbowJoint` (ArticulationBody)
- [ ] `magnet` -> `Magnet` (Transform)

Environment Objects
- [ ] `movableBox` -> `MovableBox` (Rigidbody)
- [ ] `targetZoneA` -> `TargetZoneA` (Transform)
- [ ] `targetZoneB` -> `TargetZoneB` (Transform)
- [ ] `floor` -> `Floor` (Transform)

Recommended runtime values for interactive mode
- [ ] `useRandomPositions = false`
- [ ] `showRuntimeHud = false` (optional)
- [ ] `showRuntimeGizmos = false` (optional)

### 3.2 ModelLoader fields

- [ ] `onnxModel` -> `Assets/Scripts/RobotArm.onnx` (ModelAsset)
- [ ] `robotAgent` -> same object `RobotAgentDupe`
- [ ] `backend` -> `GPUCompute` (default)
- [ ] `fixedUpdatesPerInference` -> `1`

### 3.3 ForceVisuals fields

- [ ] `robot` -> same object `RobotAgentDupe`
- [ ] `drawInReleaseBuilds` -> false (recommended)
- [ ] `torqueScale` -> keep default `0.1`

---

## 4) Inspector checklist: Zone B drag setup

Select child object `TargetZoneB`.

Expected components:
- [ ] `Transform`
- [ ] `MeshFilter`
- [ ] `MeshRenderer`
- [ ] `BoxCollider`
- [ ] `CanvasGroup`
- [ ] `BoxDrag`
- [ ] `CapsuleCollider`
- [ ] `PlacementConstraints`

### 4.1 TargetZoneB BoxDrag fields

- [ ] `targetZoneA` -> `TargetZoneA`
- [ ] `targetZoneB` -> `TargetZoneB`
- [ ] `robotAgent` -> `RobotArm/RobotAgentDupe`
- [ ] `movableBox` -> `MovableBox` Rigidbody
- [ ] `modelLoader` -> `RobotArm/ModelLoader`
- [ ] `placementConstraints` -> local `PlacementConstraints`
- [ ] `dragRole` -> `ZoneBTarget`
- [ ] `draggedZone` -> `TargetZoneB` Transform
- [ ] `disableControllersUntilPlacement` -> true (recommended)

### 4.2 TargetZoneB PlacementConstraints fields

- [ ] `xMin = -7.2`
- [ ] `xMax = 7.0`
- [ ] `zMin = -8.0`
- [ ] `zMax = 6.2`
- [ ] `radius = 6.0` (fallback only)
- [ ] `cylindricalBoundary` -> `OuterBoundary` object
- [ ] `minimumDistanceBoundary` -> `B_InnerBoundary` object
- [ ] `targetZoneA` -> `TargetZoneA`
- [ ] `targetZoneB` -> `TargetZoneB`
- [ ] `messageText` -> optional TMP label
- [ ] `dragController` -> local `BoxDrag`
- [ ] `draggedZoneIsA` -> false
- [ ] `minimumDistance` -> `1.5` (or your chosen value)
- [ ] `outerBoundary` and `innerBoundary` can be left empty if boundary transforms are assigned

---

## 5) Inspector checklist: Zone A drag setup

Select child object `TargetZoneA`.

By default this object usually has only visual/collider pieces, so add missing components:
- [ ] Add `CanvasGroup`
- [ ] Add `BoxDrag`
- [ ] Add `CapsuleCollider`
- [ ] Add `PlacementConstraints`

### 5.1 TargetZoneA BoxDrag fields

- [ ] `targetZoneA` -> `TargetZoneA`
- [ ] `targetZoneB` -> `TargetZoneB`
- [ ] `robotAgent` -> `RobotArm/RobotAgentDupe`
- [ ] `movableBox` -> `MovableBox` Rigidbody
- [ ] `modelLoader` -> `RobotArm/ModelLoader`
- [ ] `placementConstraints` -> local `PlacementConstraints`
- [ ] `dragRole` -> `ZoneAStart`
- [ ] `draggedZone` -> `TargetZoneA` Transform
- [ ] `disableControllersUntilPlacement` -> true (recommended)

### 5.2 TargetZoneA PlacementConstraints fields

- [ ] `xMin = -7.2`
- [ ] `xMax = 7.0`
- [ ] `zMin = -8.0`
- [ ] `zMax = 6.2`
- [ ] `radius = 6.0` (fallback only)
- [ ] `cylindricalBoundary` -> `OuterBoundary` object
- [ ] `minimumDistanceBoundary` -> `A_InnerBoundary` object
- [ ] `targetZoneA` -> `TargetZoneA`
- [ ] `targetZoneB` -> `TargetZoneB`
- [ ] `messageText` -> optional TMP label
- [ ] `dragController` -> local `BoxDrag`
- [ ] `draggedZoneIsA` -> true
- [ ] `minimumDistance` -> same as Zone B (`1.5` recommended)

---

## 6) Boundary objects: complete setup (Capsule colliders)

Create a parent object:
- [ ] `Boundaries`

Create child object 1:
- [ ] Name: `OuterBoundary`
- [ ] Add `CapsuleCollider`
- [ ] `Is Trigger = true`
- [ ] `Direction = Y`
- [ ] Position at floor center: `(-0.1149, floorY, -0.9234)`
- [ ] Radius: start `6.0`
- [ ] Height: any positive value (not important for this 2D XZ logic)

Create child object 2:
- [ ] Name: `A_InnerBoundary`
- [ ] Add `CapsuleCollider`
- [ ] `Is Trigger = true`
- [ ] `Direction = Y`
- [ ] Radius: `1.5` (match minimumDistance)

Create child object 3:
- [ ] Name: `B_InnerBoundary`
- [ ] Add `CapsuleCollider`
- [ ] `Is Trigger = true`
- [ ] `Direction = Y`
- [ ] Radius: `1.5` (match minimumDistance)

Important:
- [ ] Do not share one inner boundary object between both constraints.
- [ ] It is fine to share the same outer boundary object between both constraints.

---

## 7) Workspace bounds guidance

Derived from current prefab floor transform:
- Floor center near `x=-0.1149, z=-0.9234`
- Safe practical clamp values:
  - [ ] `xMin=-7.2`
  - [ ] `xMax=7.0`
  - [ ] `zMin=-8.0`
  - [ ] `zMax=6.2`

If you need tighter or wider space:
- [ ] Increase/decrease x/z min/max equally around floor center
- [ ] Keep outer boundary radius aligned with these clamps

---

## 8) Optional UI setup (Start, Reset, Timer, Quit)

Create UI:
- [ ] `Canvas`
- [ ] `TMP Text` for timer label
- [ ] Buttons: Start, Reset, Quit

Create controller:
- [ ] Empty object `UIController`
- [ ] Add `StartButton` component

StartButton fields:
- [ ] `sceneName` -> set exactly to this scene name (for example `InteractiveArm`)
- [ ] `timerText` -> your TMP timer label

Button wiring:
- [ ] Start button -> `StartButton.StartSimulation`
- [ ] Reset button -> `StartButton.ResetGame`
- [ ] Quit button -> `StartButton.QuitGame`

---

## 9) Final validation checklist

Before Play Mode:
- [ ] EventSystem exists
- [ ] Main Camera has Physics Raycaster
- [ ] ONNX ModelAsset is assigned in ModelLoader
- [ ] RobotAgentDupe references are all assigned
- [ ] Both zones have BoxDrag + PlacementConstraints configured with opposite roles
- [ ] Both constraints use same minimumDistance
- [ ] Boundaries are assigned (outer + dedicated inner per zone)

In Play Mode expected behavior:
- [ ] Zone A is draggable
- [ ] Zone B is draggable
- [ ] A cannot move inside B exclusion radius
- [ ] B cannot move inside A exclusion radius
- [ ] Invalid drag reverts to last valid position and shows red state
- [ ] Valid drag turns zone green
- [ ] On valid release, episode restarts and arm uses updated start/target logic
- [ ] Console stays free of errors

---

## 10) If something breaks, check in this order

1) Drag does not start
- Missing EventSystem
- Missing Physics Raycaster on Main Camera
- Zone missing BoxCollider or CanvasGroup

2) Zone moves but constraints do not apply
- PlacementConstraints missing references to both zones
- Wrong draggedZoneIsA value
- Boundary transforms not assigned

3) Zones always invalid
- minimumDistance too large
- Inner boundary radius too large
- Bounds too tight

4) Arm does not react after drop
- ModelLoader missing ONNX or robotAgent reference
- RobotAgentDupe references missing

5) Scene reset button does nothing
- StartButton sceneName does not match actual scene name

---

## 11) Current implementation status in this branch

Already implemented in scripts:
- Both-zone drag roles (`ZoneAStart`, `ZoneBTarget`)
- Mutual exclusion validation in PlacementConstraints
- Role-based constraints for A vs B

You still need to complete inspector wiring in scene/prefab for TargetZoneA and boundary objects.

---

When you test this, send back the first error line exactly as shown in Console and I will give you a direct fix path.
