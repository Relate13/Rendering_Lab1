#include "Cylinder.h"
#include "ResourceLayer/Factory.h"
bool Cylinder::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    auto local_ray=transform.inverseRay(ray);
    //* 2.联立方程求解
    auto A=(float)pow(local_ray.direction[0],2)+(float)pow(local_ray.direction[1],2);
    auto B=2*(local_ray.origin[0]*local_ray.direction[0]+local_ray.origin[1]*local_ray.direction[1]);
    auto C=(float)pow(local_ray.origin[0],2)+(float)pow(local_ray.origin[1],2)-(float)pow(radius,2);
    float t0,t1;
    if(!Quadratic(A,B,C,&t0,&t1))
        return false;
    //* 3.检验交点是否在圆柱范围内
    float final_t,final_phi;
    bool found= false;
    for(auto t: {t0,t1}){
        //* 3.1 检验交点是否在光线范围内
        if(t<local_ray.tNear||t>local_ray.tFar)
            continue;
        //* 3.2 检验交点是否在圆柱高度范围内
        auto p=local_ray.at(t);
        if(p[2]<0||p[2]>height)
            continue;
        //* 3.3 检验交点与圆心夹角是否符合要求
        float phi;
        if(p[1]>=0)
            phi=atan2(p[1],p[0]);
        else
            phi= 2*PI + atan2(p[1], p[0]);
        if(phi>phiMax)
            continue;
        //* 全部符合
        final_t=t,final_phi=phi,found= true;
        break;
    }
    if(!found)
        return false;
    //* 4.更新ray的tFar,减少光线和其他物体的相交计算次数
    ray.tFar=final_t;
    //* 5.填充u,v坐标
    auto p=local_ray.at(final_t);
    *u=final_phi/phiMax;
    *v=p[2]/height;
    //* Write your code here.
    return true;
}

void Cylinder::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆柱相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    auto phi=u*phiMax;
    intersection->normal=transform.toWorld(normalize({cos(phi),sin(phi),0}));
    //* 2.位置信息可以根据uv计算出，同样需要变换
    auto x=radius*cos(phi);
    auto y=radius*sin(phi);
    auto z=height*v;
    intersection->position=transform.toWorld(Point3f{x,y,z});

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

void Cylinder::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cylinder::Cylinder(const Json &json) : Shape(json) {
    radius = fetchOptional(json,"radius",1.f);
    height = fetchOptional(json,"height",1.f);
    phiMax = fetchOptional(json,"phi_max",2 * PI);
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
}

REGISTER_CLASS(Cylinder,"cylinder")
