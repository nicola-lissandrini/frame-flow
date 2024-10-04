#include "frameflow.h"

FrameData createFrameData(const std::string& parentId, const std::string& frameId, bool isStatic = true) {
    return FrameData(parentId, frameId, Transform::Identity(), Clock::now(), isStatic);
}

using namespace std;

void printPath (const FrameFlow::Path &path) {
    bool first = true;

    for (FrameFlow::FrameNode *elem : path) {
        cerr << (first ? "" : " -> ") << elem->data().frameId() << " ";
        first = false;
    }
    cerr << endl;
}

int main ()
{
    FrameFlow ff;

    auto testSubmit = [&] (auto first, auto second) {
        //cerr << "Submitting frame from " << first << " to " << second << endl;
        auto ret = ff.submitTransform(createFrameData(first, second));

        //cerr << "Submit result: " << ff.submitStatusMessage(ret) << endl;
    };

    testSubmit ("world","frame1");
    testSubmit ("world","frame2");
    testSubmit ("frame1","frame3");
    testSubmit ("frame3","frame4");
    testSubmit ("frame4","frame5");
    testSubmit ("frame5","frame6");
    testSubmit ("frame7","frame8");
    testSubmit ("frame4","frame7");

    //cerr << "Removing frame4" << endl;
    ff.removeFrame("frame4");

    testSubmit ("frame2","frame4");

    //auto tfRes = ff.lookupTransform("frame4","")

    FrameFlow::FrameNode *first = ff.getFrameNode("frame3");
    FrameFlow::FrameNode *last = ff.getFrameNode("frame6");
    auto [pathFirst, pathSecond] = ff.pathsToLCA(first, last);

    cerr << "first to LCA" << endl;
    printPath (pathFirst);
    cerr << "last to LCA" << endl;
    printPath (pathSecond);

    //cerr << ff.dump() << endl;
    cout << ff.treeToGraphviz () << endl;
}
