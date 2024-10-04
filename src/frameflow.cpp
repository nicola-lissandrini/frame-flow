#include "frameflow.h"

using namespace std;

FrameFlow::FrameFlow ():
    _frameTree(FrameData ("none",
                         "world",
                         Transform::Identity(),
                         Clock::now(),
                         true))
{
    FrameNode *root = _frameTree.root();
    _frameNodeTable.insert({root->data().frameId(), root});
}

FrameFlow::FrameNode *FrameFlow::getFrameNode (const string &frameId)
{
    if (_frameNodeTable.count(frameId) > 0)
        return _frameNodeTable[frameId];
    return nullptr;
}

FrameFlow::SubmitStatus FrameFlow::submitTransform (const string &parentId,
                                           const string &frameId,
                                           const Transform &transform,
                                           const Time &timestamp,
                                           bool isStatic)
{
    // Note: transform is deep copied but we'll live with that for now
    return submitTransform(FrameData (parentId, frameId, transform, timestamp, isStatic));
}

bool FrameFlow::moveToPending (FrameData &&frameData)
{
    auto existingIt = _pendingFrames.find(frameData.frameId());

    // in case received frame is already pending, update if parent matches
    if (existingIt != _pendingFrames.end()) {
        const FrameData &existingFrameData = existingIt->second;
        if (existingFrameData.parentId() != frameData.parentId())

        return false;
    }

    _pendingFrames.emplace (make_pair (frameData.frameId(), std::move(frameData)));
    return true;
}

FrameFlow::SubmitStatus FrameFlow::submitTransform (FrameData &&frameData)
{
    FrameNode *parentNode = getFrameNode (frameData.parentId());


    // If parent does not exists, add the data to the pending frames
    if (parentNode == nullptr) {
        if (!moveToPending (std::move (frameData)))
            return SubmitStatus::UNMATCHED_PARENT;

        return SubmitStatus::NO_ROUTE_TO_WORLD;
    }

    FrameNode *currentNode = getFrameNode (frameData.frameId());

    // if frameId does not exist yet, create it and set data
    if (currentNode == nullptr) {
        FrameNode *newFrameNode = createChildNode (parentNode, std::move (frameData));
        return SubmitStatus::ADDED_NEW;
    }

    // if it exists, check if its correctly a parentNode child
    for (FrameNode *child : parentNode->children()) {
        if (child->data().frameId() == frameData.frameId()) {
            updateFrameNode (child, std::move(frameData));
            return SubmitStatus::UPDATED_EXISTING;
        }
    }

    // if both parents and child already exists but with incorrect relation
    //cerr << "Trying to add child frame " << frameData.frameId() << " to parent " << frameData.parentId()
    //     << "but child frame has already parent " << currentNode->parent()->data().frameId() << endl;

    return SubmitStatus::UNMATCHED_PARENT;
}

void FrameFlow::updateFrameNode (FrameNode *frameNode, FrameData &&frameData) {
    frameNode->data() = std::move (frameData);
}

FrameFlow::FrameNode *FrameFlow::createChildNode  (FrameNode *parentNode, FrameData &&frameData)
{
    FrameNode *newFrameNode = parentNode->addChild(std::move (frameData));

    _frameNodeTable[newFrameNode->data().frameId()] = newFrameNode;

    recheckPendingFrames ();
    return nullptr;
}

void FrameFlow::recheckPendingFrames ()
{
    decltype(_pendingFrames)::iterator it;
    FrameNode *currentParent;

    for (it = _pendingFrames.begin(); it != _pendingFrames.end(); it++) {
        const FrameData &currentFrameData = it->second;
        currentParent = getFrameNode (currentFrameData.parentId());

        if (currentParent != nullptr)
            break;
    }

    if (it != _pendingFrames.end()) {
        FrameData &validFrameData = it->second;

        _pendingFrames.erase(it);

        /* Warning! this correctly cause a continuous loop
         * until all valid pending frames are added, but
         * it increases the call stack linearly with the number
         * of found transform. It shouldn't be a problem now
         * but keep an eye on it
        */
        createChildNode (currentParent, std::move(validFrameData));
    }
}

FrameFlow::RemovalResult FrameFlow::removeFrame (const std::string &frameId)
{
    FrameNode *frameNode = getFrameNode (frameId);

    if (frameNode == nullptr)
        return RemovalResult::FRAME_NOT_FOUND;

    _frameTree.traverse (FrameTree::DEPTH_FIRST_POSTORDER, frameNode, [this, frameNode] (FrameNode *currentNode) {
        _frameNodeTable.erase(currentNode->data().frameId());
        if (currentNode != frameNode)
            moveToPending(std::move (currentNode->data()));
    });

    frameNode->parent()->removeChildByPointer (frameNode);

    return RemovalResult::OK;
}


pair<FrameFlow::Path, FrameFlow::Path> FrameFlow::pathsToLCA (FrameNode *baseFrameNode,
                                                                                   FrameNode *targetFrameNode)
{
    auto equalizeLevels = [] (FrameNode *startNode, int targetDepth, Path &chainOutput) -> FrameNode * {
        FrameNode *current = startNode;

        while (current->depth() > targetDepth) {
            chainOutput.push_back(current);

            current = current->parent();
        }

        return current;
    };

    auto parallelAscend = [] (FrameNode *startBTL,
                              FrameNode *startLTT,
                              Path &pathBTL,
                              Path &pathLTT) {
        FrameNode *currentBTL = startBTL;
        FrameNode *currentLTT = startLTT;

        while (currentBTL != currentLTT) {
            pathBTL.push_back(currentBTL);
            pathLTT.push_back(currentLTT);

            currentBTL = currentBTL->parent();
            currentLTT = currentLTT->parent();
        }
    };

    // BTL: Base To LCA (Lowest Common Ancestor)
    // LTT: LCA To Target
    Path pathBTL, pathLTT;
    FrameNode *currentBTL, *currentLTT;

    // Only one of the two will produce a non-empty path
    currentBTL = equalizeLevels (baseFrameNode, targetFrameNode->depth(), pathBTL);
    currentLTT = equalizeLevels (targetFrameNode, baseFrameNode->depth(), pathLTT);

    parallelAscend (currentBTL, currentLTT, pathBTL, pathLTT);

    return {pathBTL, pathLTT};
}

bool FrameFlow::frameExpired (const FrameData &frameData) {
    return Clock::now () > frameData.timestamp() + _params.expireThreshold;
}

FrameFlow::TransformResult FrameFlow::lookupTransform (const std::string &baseFrameId,
                                                      const std::string &targetFrameId)
{
    FrameNode *baseFrameNode = getFrameNode (baseFrameId);

    if (baseFrameNode == nullptr)
        return LookupStatus::NO_BASE_FRAME;

    FrameNode *targetFrameNode = getFrameNode (targetFrameId);

    if (targetFrameNode == nullptr)
        return LookupStatus::NO_TARGET_FRAME;

    auto [pathBTL, pathLTT] = pathsToLCA (baseFrameNode, targetFrameNode);

    TransformResult chainBTLResult, chainLTTResult;

    auto chainPathTransforms = [this] (const auto &begin, const auto &end) -> TransformResult {
        Transform chain = Transform::Identity();
        bool expired = false;

        for_each (begin, end, [&] (FrameNode *current) {
            const FrameData &currentFrameData = current->data();
            if (!currentFrameData.isStatic() && frameExpired (currentFrameData))
                expired = true;
            chain = chain * current->data().transform();
        });

        if (expired)
            return LookupStatus::EXPIRED_CHAIN;
        else
            return chain;
    };

    // the first one needs to be iterated in order (which is botttom up)
    chainBTLResult = chainPathTransforms (pathBTL.begin(), pathBTL.end());
    // The second in reverse (which is top down)
    chainLTTResult = chainPathTransforms (pathLTT.rbegin(), pathLTT.rend());

    if (!chainBTLResult.success() || !chainLTTResult.success())
        return LookupStatus::EXPIRED_CHAIN;

    return chainBTLResult.value().inverse() * chainLTTResult.value();
}

string FrameFlow::dump ()
{
    string tree = _frameTree.toJson([](FrameNode *current) {
        stringstream ss;
        ss << "{\"frame_id\": \"" << current->data().frameId() << "\", \"parent_id\": \"" << current->data().parentId() << "\"}";
        return ss.str ();
    });

    stringstream framesMap;

    framesMap << "\"frames_map\": [";
    bool first = true;
    for (auto [frameId, frameNode]: _frameNodeTable) {
        framesMap << (first ? "\"" : ", \"") << frameNode->data().frameId() << "\"";
        first = false;
    }

    framesMap << "]";

    stringstream pendingMap;

    pendingMap << "\"pending_map\": [";
    first = true;
    for (auto [frameId, frameData] : _pendingFrames) {
        pendingMap << (first ? "" : ", ") << "{\"frame_id\": \""
                   << frameData.frameId()  << "\", \"parent_id\": \""
                   << frameData.parentId() << "\"}";
        first = false;
    }
    pendingMap << "]";

    stringstream total;

    total << "{\"tree\": " << tree << ", " << framesMap.str() << ", " << pendingMap.str() << "}";

    return total.str();

}











