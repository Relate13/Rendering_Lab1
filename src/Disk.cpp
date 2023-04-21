#include "Disk.h"
#include "ResourceLayer/Factory.h"
bool Disk::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆环的相交 填充primId,u,v.如果相交，更新光线的tFar
    *primID=0;
    //* 1.光线变换到局部空间
    auto local_ray = transform.inverseRay(ray);
    //* 2.判断局部光线的方向在z轴分量是否为0
    if(local_ray.direction[2]==0)
        return false;//Ambiguous Here, May cause a problem
    //* 3.计算光线和平面交点
    auto t=(0-local_ray.origin[2])/(local_ray.direction[2]);
    //* 4.检验交点是否在圆环内
    //* 4.1 检验交点是否在光线范围内
    if(t<local_ray.tNear||t>local_ray.tFar)
        return false;//Ambiguous Here, May cause a problem
    //* 4.2 检验交点是否在圆环半径范围内
    auto intersected_point=local_ray.at(t);
    // distance = sqrt(x^2+y^2)
    auto distance_to_origin=(float)sqrt(pow(intersected_point[0],2)+pow(intersected_point[1],2));
    if(distance_to_origin<innerRadius||distance_to_origin>radius)
        return false;//Ambiguous Here, May cause a problem
    //* 4.3 检验交点是否在圆环角度范围内
    float phi;
    if(intersected_point[1]>=0)
        phi=atan2(intersected_point[1],intersected_point[0]);
    else
        phi= 2*PI + atan2(intersected_point[1], intersected_point[0]);
    if(phi>phiMax)
        return false;
    //* 5.更新ray的tFar,减少光线和其他物体的相交计算次数
    ray.tFar = t;
    //* 6.填充u,v坐标
    *u=phi/phiMax;
    *v=(distance_to_origin-innerRadius)/(radius-innerRadius);
    //* Write your code here.
    return true;
}

void Disk::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆环相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    intersection->normal=transform.toWorld(normalize({0,0,1}));
    //* 2.位置信息可以根据uv计算出，同样需要变换
    auto phi=u*phiMax;
    auto distance_to_origin=v*(radius-innerRadius)+innerRadius;
    float x=distance_to_origin * cos(phi);
    float y=distance_to_origin * sin(phi);
    intersection->position=transform.toWorld(Point3f{x,y,0});
    //* Write your code here.
    /// ----------------------------------------------------


    intersection->shape = this;
    intersection->distance = distance;
    intersection->texCoord = Vector2f{u, v};
    Vector3f tangent{1.f, 0.f, .0f};
    Vector3f bitangent;
    if (std::abs(dot(tangent, intersection->normal)) > .9f) {
        tangent = Vector3f(.0f, 1.f, .0f);
    }
    bitangent = normalize(cross(tangent, intersection->normal));
    tangent = normalize(cross(intersection->normal, bitangent));
    intersection->tangent = tangent;
    intersection->bitangent = bitangent;
}

Disk::Disk(const Json &json) : Shape(json) {
//    normal = transform.toWorld(Vector3f(0,0,1));
//    origin = transform.toWorld(Point3f(0,0,0));
//    auto
//    //radius认为是三个方向的上的scale平均
//    vecmat::vec4f v(1,1,1,0);
//    auto radiusVec = transform.scale * v;
//    radiusVec/=radiusVec[3];
//    radius = (radiusVec[0]+radiusVec[1]+radiusVec[2])/3;
     radius = fetchOptional(json,"radius",1.f);
     innerRadius = fetchOptional(json,"inner_radius",0.f);
     phiMax = fetchOptional(json,"phi_max",2 * PI);
     AABB local(Point3f(-radius,-radius,0),Point3f(radius,radius,0));
     boundingBox = transform.toWorld(local);
}

void Disk::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {
        //采样光源 暂时不用实现
}
REGISTER_CLASS(Disk, "disk")

