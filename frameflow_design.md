# Design aspects of **FrameFlow**

## **Overview**

**FrameFlow** is a system designed to handle frame transformations in robotics, serving as an alternative to tf2 with an emphasis on clarity, robustness, and efficiency. It manages coordinate frames and the transformations between them, allowing for chaining and querying of transforms within a structured framework.

## **Key Components**

1. **Centralized Node:**
   - Acts as the single source of truth for all transforms.
   - Receives and processes incoming transforms.
   - Handles computation of chained transforms between frames.
   - Simplifies development by avoiding the complexities of a distributed system.

2. **Frames and Transforms:**
   - **Frames** represent coordinate systems, each associated with a unique `frame_id`.
   - **Transforms** define the relationship between two frames, including rotation and translation.
   - Each transform is from a **parent frame** to a **child frame**.

3. **Tree Structure:**
   - Frames are organized in a tree hierarchy with a fixed root frame called `"world"`.
   - Each frame maintains:
     - A pointer to its **parent** frame.
     - A list of its **child** frames.
     - The transform from its parent to itself.
   - All frames are stored in an **unordered_map** for efficient access, keyed by `frame_id`.

4. **Transforms Chaining:**
   - To compute the transform between two frames, FrameFlow chains the transforms along the path connecting them.
   - The chaining is done in order: for frames `A`, `B`, and `C`, the transform from `A` to `C` is computed as:
     \[
     T_{A,C} = T_{A,B} \times T_{B,C}
     \]
   - Multiplication is performed in sequence, respecting the parent-child relationships.

## **Algorithms and Strategies**

1. **Adding Transforms:**
   - When a new transform is received:
     - If the parent frame exists:
       - The child frame is added or updated in the tree as a child of the parent.
     - If the parent frame does not exist:
       - The transform is added to an **orphan list** for deferred processing.
   - The system ensures that each frame has only one parent, preventing multiple parent assignments.

2. **Orphan Handling:**
   - Orphan transforms (where the parent frame is missing) are stored temporarily.
   - Whenever a new frame is added to the tree, the orphan list is re-checked to see if any orphans can now be integrated.
   - This process may require iterating over the orphan list multiple times until no more orphans can be added to the tree.
   - This strategy handles out-of-order arrival of transforms and ensures the tree is built correctly.

3. **Cycle Prevention:**
   - To avoid circular dependencies:
     - When adding a transform, the system checks if the child frame already exists in the tree.
     - If the child exists but is not a direct child of the specified parent, the transform is rejected, and an error is issued.
   - This check ensures that frames cannot be reassigned to new parents if it would create a cycle, maintaining the integrity of the tree structure.

4. **Transform Lookup (`lookupTransform`):**
   - When a transform between two frames is requested:
     - The system traverses the tree from the source frame to the target frame.
     - It multiplies the transforms along the path to compute the overall transformation.
     - If any transform along the path is outdated (exceeds the safety threshold), the lookup fails, and an error is returned.
   - This ensures that only valid and up-to-date transforms are used in computations.

5. **Handling Missing or Outdated Transforms:**
   - Transforms are not automatically removed if they become outdated.
   - If a transform is missing or outdated, the system considers the chain invalid but retains the structural relationship.
   - This approach avoids unnecessary removal of edges due to temporary issues (e.g., sensor occlusion).

6. **Manual Removal of Frames and Edges:**
   - Changes to the topology (removing frames or edges) require manual intervention.
   - When a frame is manually removed:
     - All its descendants are detached from the tree and added to the orphan list.
     - They are fully released from previous parent-child relationships.
   - This ensures that descendants can be safely re-integrated into the tree if appropriate transforms become available.

7. **Tree Integrity Maintenance:**
   - The tree structure is maintained by enforcing that each frame has exactly one parent (except for the root).
   - Parent-child relationships are strictly managed to prevent inconsistencies.
   - The system disallows reassigning a frame to a different parent without explicit removal and reattachment.

## **Design Considerations**

1. **Single-Threaded Operation:**
   - FrameFlow operates in a single thread, avoiding concurrency issues.
   - Incoming transforms are queued and processed sequentially.

2. **Performance Optimization:**
   - The use of an unordered_map allows for constant-time access to frames.
   - Traversal algorithms are optimized for efficiency.
   - Caching strategies may be considered if performance becomes a concern, though not initially necessary for small to medium-sized trees.

3. **Safety Threshold for Transforms:**
   - A user-configurable parameter defines how long a transform is considered valid.
   - Outdated transforms invalidate the chain but do not alter the tree's structure.

4. **Time Synchronization:**
   - FrameFlow assumes that timestamps of incoming transforms are synchronized externally.
   - This adheres to the single-responsibility principle, keeping time management outside the scope of FrameFlow.

5. **Error Handling:**
   - Clear error messages are provided when transforms cannot be added or when lookups fail.
   - This aids in debugging and ensures users are aware of issues in the frame relationships.

## **Potential Edge Cases and Solutions**

1. **Transforms Arriving Out of Order:**
   - The orphan list mechanism ensures that transforms can be integrated once their parent frames become available.

2. **Circular Dependencies:**
   - The parent-existence check prevents cycles by disallowing a frame from being added as a child if it already exists elsewhere in the tree.

3. **Removal of Frames with Descendants:**
   - When a frame is removed, its descendants are orphaned and detached, preventing unintended reattachment with outdated relationships.

4. **Dynamic Topology Changes:**
   - Frequent additions or removals of frames are handled gracefully.
   - The system expects topology changes to be relatively infrequent due to physical constraints.

## **Conclusion**

FrameFlow provides a robust and efficient framework for managing frame transformations in robotic systems. By employing a centralized node, maintaining a strict tree structure, and utilizing strategies like orphan handling and cycle prevention, FrameFlow ensures accurate and reliable computation of chained transforms. The design emphasizes simplicity, clarity, and explicit control over topology changes, making it a powerful tool for developers working with complex frame relationships.

