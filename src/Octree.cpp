#include "Octree.h"
#include <queue>

struct Octree::OctreeNode {
    AABB boundingBox;
    std::shared_ptr<OctreeNode> subNodes[8];
    int primCount = -1;
    int primIdxBuffer[ocLeafMaxSize]{};

    //for leaf nodes
    explicit OctreeNode() = default;

    explicit OctreeNode(const std::vector<int> &_primIdxBuffer, const AABB &aabb) {
        boundingBox = aabb;
        primCount = (int) _primIdxBuffer.size();
        for (int i = 0; i < primCount; ++i) {
            primIdxBuffer[i] = _primIdxBuffer.at(i);
        }
    }
};

std::vector<AABB> getSubBoxes(const AABB &aabb) {
    std::vector<AABB> result;
    Vector3f distance=(aabb.pMax-aabb.pMin)/2;
    auto current=aabb.pMin;
    for(int i=0;i<=1;++i){
        current[0]+=(float)i*distance[0];
        for(int j=0;j<=1;++j){
            current[1]+=(float)j*distance[1];
            for(int k=0;k<=1;++k){
                current[2]+=(float)k*distance[2];
                result.emplace_back(current,current+distance);
                current[2]-=(float)k*distance[2];
            }
            current[1]-=(float)j*distance[1];
        }
        current[0]-=(float)i*distance[0];
    }
    return result;
}

std::shared_ptr<Octree::OctreeNode> Octree::recursiveBuild(const AABB &aabb,
                                                   const std::vector<int> &primIdxBuffer) {
    //* todo 完成递归构建八叉树
    //* 构建方法请看实验手册
    //* 要注意的一种特殊是当节点的某个子包围盒和当前节点所有物体都相交，我们就不用细分了，当前节点作为叶子节点即可。

    //* 节点数足够少
    if (primIdxBuffer.size() <= ocLeafMaxSize)
        return std::make_shared<OctreeNode>(primIdxBuffer, aabb);

    //* 划分空间
    auto subBoxes = getSubBoxes(aabb);
    //* 划分每个空间的primIdx数组
    std::vector<std::vector<int>> subBuffers(8);
    //* 记录空白空间的数量
    int empty = 0;
    //* 对于每一个划分出的空间
    for (int i = 0; i < 8; ++i) {
        for (auto primIdx: primIdxBuffer) {
            // 如果任意图元与当前空间相交，就把它加入当前空间的primIdx数组中
            if (shapes[primIdx]->getAABB().Overlap(subBoxes.at(i)))
                subBuffers.at(i).push_back(primIdx);
        }
        if (subBuffers.at(i).empty())//如果当前空间的primIdx数组为空，说明该空间没有与任何图元相交
            ++empty;
    }
//    //TODO:如何处理特殊情况我不好说，暂时先不处理
//    if(empty==7&&primIdxBuffer.size()<=ocLeafMaxSize){//如果所有图元只在一个子空间中并且空间能够装下所有图元，直接把当前节点作为叶节点返回
//        return std::make_shared<OctreeNode>(primIdxBuffer, aabb);
//    }
    //* 新建节点
    auto node = std::make_shared<OctreeNode>();
    node->boundingBox = aabb;
    //* 对每个子空间递归建树
    for (int i = 0; i < 8; ++i) {
        node->subNodes[i] = recursiveBuild(subBoxes[i], subBuffers[i]);
    }
    return node;
}


void Octree::build() {
    //* 首先计算整个场景的范围
    for (const auto &shape: shapes) {
        //* 自行实现的加速结构请务必对每个shape调用该方法，以保证TriangleMesh构建内部加速结构
        //* 由于使用embree时，TriangleMesh::getAABB不会被调用，因此出于性能考虑我们不在TriangleMesh
        //* 的构造阶段计算其AABB，因此当我们将TriangleMesh的AABB计算放在TriangleMesh::initInternalAcceleration中
        //* 所以请确保在调用TriangleMesh::getAABB之前先调用TriangleMesh::initInternalAcceleration
        shape->initInternalAcceleration();

        boundingBox.Expand(shape->getAABB());
    }

    //* 构建八叉树
    std::vector<int> primIdxBuffer(shapes.size());
    std::iota(primIdxBuffer.begin(), primIdxBuffer.end(), 0);
    root = recursiveBuild(boundingBox, primIdxBuffer);
}

bool Octree::rayIntersect(Ray &ray, int *geomID, int *primID,
                          float *u, float *v) const {
    //*todo 完成八叉树求交
    bool result = false;
    std::queue<std::shared_ptr<OctreeNode>> job{};
    job.push(root);
    while (!job.empty()) {
        auto currentNode = job.front();
        job.pop();
        if (currentNode->boundingBox.RayIntersect(ray)) {
            if (currentNode->primCount == -1) {
                //primCount为-1，必为树枝，把树枝下的子节点都加入工作集中
                for (const auto &subNode: currentNode->subNodes) {
                    job.push(subNode);
                }
            } else if (currentNode->primCount > 0) {
                //primCount大于0，说明为叶子节点且存在相交可能，进行求交判断
                for (int i = 0; i < currentNode->primCount; ++i) {
                    int index = currentNode->primIdxBuffer[i];
                    bool hit = shapes[index]->rayIntersectShape(ray, primID, u, v);
                    if (hit)
                        *geomID = index;
                    result = hit || result;
                }
            }
        }
    }
    return result;
}