# FrameFlow System

FrameFlow is a centralized system for managing frame transformations in a robotics environment. Inspired by the concept of `tf2` in ROS, FrameFlow offers a more practical and streamlined approach for handling transformation trees while respecting frame conventions, addressing some of the challenges of using `tf2`.

## Overview

FrameFlow is designed to manage relationships between different coordinate frames in a robotics system, providing accurate and efficient transformations between those frames. It allows developers to understand and work with how different components or sensors relate to each other in space. The system maintains a central tree structure of frame relationships, making it an ideal choice for robotic systems that require up-to-date, reliable transformations in real-time applications.

The core concepts and key features of FrameFlow are:

### Centralized Frame Management

FrameFlow provides a convenient way to manage frame transformations in a centralized manner. This approach maintains consistency across the system, reduces complexity, and helps ensure reliable spatial relationships in real-time robotic systems.

### Tree Data Structure

FrameFlow represents the relationships between frames as a tree, starting with a root frame known as the "world" frame. All other frames are added as children of existing frames, creating a clear hierarchy. This structure allows for efficient traversal and manipulation, enabling easy computation of transformations between any two frames.

### Dynamic Frame Updates

Frames can be dynamically added to the system, and relationships can be modified or removed. If a frame is added whose parent does not yet exist, the frame is moved to a pending list, where it waits until the parent becomes available. This dynamic handling of frames makes the system robust to changes and adaptable for complex, evolving environments.

### Key Features

1. **Automatic Transform Chaining**: FrameFlow computes transformations between any two frames by automatically chaining individual transforms, ensuring accurate spatial relationships.
2. **Support for Static and Dynamic Frames**: Static transformations are maintained permanently, while dynamic ones are continuously updated and are expected to be updated within a set threshold, after which the entire chain is disabled.
3. **Single Source of Truth**: Centralized management ensures consistency in frame transformations, reducing the risk of conflicts or errors.
4. **Dynamic Handling of Frames**: Frames with missing parents are placed in a pending list until they can be appropriately added to the tree structure.

These features make FrameFlow particularly effective in robotic systems where spatial relationships between components and sensors need to be managed accurately and efficiently.

## Public API

The following public methods are available to interact with the FrameFlow system:

### 1. `submitTransform`

```cpp
SubmitStatus submitTransform(const std::string &parentId, const std::string &frameId, const Transform &transform, const Time &timestamp, bool isStatic = false);
```

Submits a new transformation between a parent and child frame.

- **Parameters**:
  - `parentId`: The identifier of the parent frame.
  - `frameId`: The identifier of the child frame.
  - `transform`: The transformation from the parent frame to the child frame.
  - `timestamp`: The time at which the transformation is valid.
  - `isStatic`: A boolean indicating whether the transformation is static.
- **Behavior**:
  - If the parent frame exists, the child frame is added or updated in the tree.
  - If the parent frame does not exist, the frame is moved to the pending list until the parent becomes available.
- **Return Values**:
  - `ADDED_NEW`: A new frame has been added to the tree.
  - `UPDATED_EXISTING`: An existing frame has been updated.
  - `NO_ROUTE_TO_WORLD`: The frame has been added to the pending list because its parent is not available.
  - `UNMATCHED_PARENT`: The frame has a parent that conflicts with an existing frame in the pending list.

### 2. `lookupTransform`

```cpp
TransformResult lookupTransform(const std::string &baseFrameId, const std::string &targetFrameId);
```

Looks up the transformation between two frames. If the frames are connected, the function calculates the transformation by chaining the individual transforms from the base frame to the target frame.

- **Parameters**:
  - `baseFrameId`: The identifier of the base frame.
  - `targetFrameId`: The identifier of the target frame.
- **Behavior**:
  - Finds the lowest common ancestor (LCA) of the two frames.
  - Chains the transforms from the base frame to the LCA, and from the LCA to the target frame.
  - Returns the combined transformation or an error if the chain is invalid (e.g., expired transforms).
- **Return Values**:
  - A valid `Transform` if the frames are connected and not expired.
  - `NO_BASE_FRAME`: The base frame does not exist in the tree.
  - `NO_TARGET_FRAME`: The target frame does not exist in the tree.
  - `EXPIRED_CHAIN`: One or more transformations in the chain are expired.

### 3. `removeFrame`

```cpp
RemovalResult removeFrame(const std::string &frameId);
```

Removes a frame from the tree.

- **Parameters**:
  - `frameId`: The identifier of the frame to be removed.
- **Behavior**:
  - The specified frame is removed from the tree.
  - All children of the removed frame are moved to the pending list to be reconnected once a valid parent becomes available.
- **Return Values**:
  - `OK`: The frame has been successfully removed.
  - `FRAME_NOT_FOUND`: The specified frame was not found in the tree.

## Examples

### Submitting a Transformation

```cpp
FrameFlow frameFlow;
frameFlow.submitTransform("world", "robot_base", Transform::Identity(), Clock::now(), true);
```

This submits a static transformation from the "world" frame to the "robot_base" frame, indicating that the transformation is always valid and does not expire.

### Looking Up a Transformation

```cpp
auto result = frameFlow.lookupTransform("world", "robot_arm");
if (result.success()) {
    Transform transform = result.value();
    // Use the transform here
} else {
    // Handle error: result.status()
}
```

This example looks up the transformation between the "world" frame and the "robot_arm" frame. If successful, it returns the combined transformation.

### Removing a Frame

```cpp
auto removalStatus = frameFlow.removeFrame("robot_base");
if (removalStatus == FrameFlow::RemovalResult::OK) {
    // Frame successfully removed
} else {
    // Handle error: frame not found
}
```

This removes the "robot_base" frame from the tree, moving any of its children to the pending list.

### Handling Dynamic Transformations

```cpp
// Submit a dynamic transform between two frames
frameFlow.submitTransform("robot_base", "sensor_frame", Transform::Translation(1.0, 0.0, 0.0), Clock::now(), false);

// Attempt to look up the transform after some time
auto dynamicResult = frameFlow.lookupTransform("world", "sensor_frame");
if (dynamicResult.status() == FrameFlow::LookupStatus::EXPIRED_CHAIN) {
    // The transformation has expired; handle the situation accordingly
}
```

In this example, a dynamic transformation is submitted between the "robot_base" and "sensor_frame". The system keeps track of the timestamp to determine if the transformation is still valid.

## Summary

FrameFlow provides an efficient, centralized way to manage frame transformations in robotics systems. By utilizing a tree structure and a pending list for frames with unavailable parents, FrameFlow ensures that frame transformations are both consistent and reliable.

The three main functions (`submitTransform`, `lookupTransform`, and `removeFrame`) allow users to add, query, and remove transformations, ensuring flexibility and ease of use. This system is especially useful in scenarios where accurate frame relationships are crucial for robotic operations and sensor data fusion.

FrameFlow aims to be a robust, modular tool to help robotics developers manage frame transformations effectively, providing real-time consistency and enabling complex frame-based reasoning with ease.