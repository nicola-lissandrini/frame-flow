#ifndef FRAMEFLOW_H
#define FRAMEFLOW_H

#include <nlib2/nl_utils.h>
#include <nlib2/nl_tree.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using Transform = Eigen::Transform<float, NLIB_3D, Eigen::Isometry>;
template<int rows, int cols>
using Matrix = Eigen::Matrix<float, rows, cols>; // col major
using Matrix2X = Matrix<NLIB_3D, Eigen::Dynamic>;
using VectorX = Matrix<Eigen::Dynamic, 1>;
using Vector3 = Matrix<NLIB_3D, 1>;

using Rotation = Eigen::Quaternionf;
using Translation = Vector3;
using Point = Vector3;

using Clock = std::chrono::system_clock;
using Time = Clock::time_point;
using Duration = Time::duration;

class FrameData
{
public:
    FrameData (const std::string &parentId,
              const std::string &frameId,
              const Transform &transform,
              const Time &timestamp,
              bool isStatic):
        _parentId(parentId),
        _frameId(frameId),
        _transform(transform),
        _timestamp(timestamp),
        _isStatic(isStatic)
    {}

    FrameData (const FrameData &) = default;
    FrameData (FrameData &&) = default;
    FrameData& operator = (const FrameData &) = default;
    FrameData& operator = (FrameData &&) = default;

    const std::string &parentId() const { return _parentId; }
    const std::string &frameId() const { return _frameId; }
    const Transform &transform() const { return _transform; }
    const Time &timestamp() const { return _timestamp; }
    bool isStatic() const { return _isStatic; }

private:
    std::string _parentId;
    std::string _frameId;
    Transform _transform;
    Time _timestamp;
    bool _isStatic;
};

class FrameFlow
{
    constexpr static const char *LOOKUP_STATUS_STRS[] = {
        "OK",
        "NO_BASE_FRAME",
        "NO_TARGET_FRAME",
        "EXPIRED_CHAIN"
    };

    constexpr static const char *SUBMIT_STATUS_STRS[] = {
        "ADDED_NEW",
        "UPDATED_EXISTING",
        "NO_ROUTE_TO_WORLD",
        "UNMATCHED_PARENT"
    };

    constexpr static const char *REMOVAL_RESULT_STRS[] = {
        "OK",
        "FRAME_NOT_FOUND"
    };

public:
    enum class LookupStatus {
        OK,
        NO_BASE_FRAME,
        NO_TARGET_FRAME,
        EXPIRED_CHAIN
    };

    enum class SubmitStatus {
        ADDED_NEW,
        UPDATED_EXISTING,
        NO_ROUTE_TO_WORLD,
        UNMATCHED_PARENT
    };

    enum class RemovalResult {
        OK,
        FRAME_NOT_FOUND
    };


    using FrameNode = nlib2::TreeNode<FrameData>;
    using FrameTree = nlib2::Tree<FrameData>;
    using TransformResult = nlib2::AlgorithmResult<Transform, LookupStatus, LOOKUP_STATUS_STRS, LookupStatus::OK>;
    using Path = std::list<FrameNode *>;

    struct Params {
        Duration expireThreshold;
    };

    FrameFlow ();

    void setParams (const Params &params) { _params = params; }

    SubmitStatus submitTransform (const std::string &parentId,
                                    const std::string &frameId,
                                    const Transform &transform,
                                    const Time &timestamp,
                                    bool isStatic = false);

    TransformResult lookupTransform (const std::string &baseFrameId,
                                    const std::string &targetFrameId);

    RemovalResult removeFrame (const std::string &frameId);

    std::string dump();
    std::string treeToGraphviz () {
        return _frameTree.toGraphviz<std::string> ([](FrameNode *node) -> std::string {
            return "\"" /*+ node->data().parentId() + " to "*/ + node->data().frameId() + "\"";
        });
    }
    std::string submitStatusMessage (SubmitStatus status) {
        return SUBMIT_STATUS_STRS[static_cast<int> (status)];
    }
    std::string removeResultMessage (SubmitStatus status) {
        return SUBMIT_STATUS_STRS[static_cast<int> (status)];
    }

    // private:
    SubmitStatus submitTransform (FrameData &&frameData);
    FrameNode *getFrameNode (const std::string &frameId);
    FrameNode *createChildNode (FrameNode *parent, FrameData &&frameData);
    bool moveToPending (FrameData &&frameData);
    void updateFrameNode (FrameNode *frameNode, FrameData &&frameData);
    void recheckPendingFrames ();
    // find paths to Lowest Common Ancestor
    std::pair<Path, Path> pathsToLCA (FrameNode *baseFrameNode, FrameNode *targetFrameNode);
    bool frameExpired (const FrameData &frameData);



private:
    Params _params;
    FrameTree _frameTree;
    // keep a list of nodes indexed by frame id for fast retreival
    std::unordered_map<std::string, FrameNode *> _frameNodeTable;
    // Store received frame data for which the parent id is not received yet
    std::unordered_map<std::string, FrameData> _pendingFrames;
};

#endif // FRAMEFLOW_H
