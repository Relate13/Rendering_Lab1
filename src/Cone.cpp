#include "Cone.h"
#include "ResourceLayer/Factory.h"

bool Cone::rayIntersectShape(Ray &ray, int *primID, float *u, float *v) const {
    //* todo 完成光线与圆柱的相交 填充primId,u,v.如果相交，更新光线的tFar
    //* 1.光线变换到局部空间
    auto local_ray=transform.inverseRay(ray);
    //* 2.联立方程求解
    auto D=local_ray.direction;
    auto V=Vector3f{0, 0, -1};
    auto point_C=Point3f{0, 0, height};
    auto CO= local_ray.origin-point_C;
    //auto theta=atan2(radius,height);
    auto A= (float)pow(dot(D, V), 2) - (float)pow(cosTheta, 2);
    auto B=2*(dot(D, V) * dot(CO, V) - dot(D, CO) * (float) pow(cosTheta, 2));
    auto C=(float)pow(dot(CO,V),2)-
            dot(CO,CO)*(float)pow(cosTheta,2);
    float t0,t1;
    if(!Quadratic(A,B,C,&t0,&t1))
        return false;
    //* 3.检验交点是否在圆锥范围内
    float final_t,final_phi;
    bool found= false;
    for(auto t: {t0,t1}){
        //* 3.1 检验交点是否在光线范围内
        if(t<local_ray.tNear||t>local_ray.tFar)
            continue;
        //* 3.2 检验交点是否在圆锥高度范围内
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
    return true;
}

void Cone::fillIntersection(float distance, int primID, float u, float v, Intersection *intersection) const {
    /// ----------------------------------------------------
    //* todo 填充圆锥相交信息中的法线以及相交位置信息
    //* 1.法线可以先计算出局部空间的法线，然后变换到世界空间
    //* 2.位置信息可以根据uv计算出，同样需要变换
    auto phi=phiMax*u;
    auto M_radius=radius*(1-v);
    auto x=M_radius*cos(phi);
    auto y=M_radius* sin(phi);
    auto z=height*v;
    auto M=Point3f{x,y,z};
    intersection->position=transform.toWorld(M);
    auto K=Point3f{0,0,height-(height-z)/(float)pow(cosTheta,2)};
    auto MK=M-K;
    intersection->normal=transform.toWorld(normalize(MK));

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

void Cone::uniformSampleOnSurface(Vector2f sample, Intersection *result, float *pdf) const {

}

Cone::Cone(const Json &json) : Shape(json) {
    radius = fetchOptional(json, "radius", 1.f);
    height = fetchOptional(json, "height", 1.f);
    phiMax = fetchOptional(json, "phi_max", 2 * PI);
    float tanTheta = radius / height;
    cosTheta = sqrt(1/(1+tanTheta * tanTheta));
    //theta = fetchOptional(json,)
    AABB localAABB = AABB(Point3f(-radius,-radius,0),Point3f(radius,radius,height));
    boundingBox = transform.toWorld(localAABB);
    boundingBox = AABB(Point3f(-100,-100,-100),Point3f(100,100,100));
}

REGISTER_CLASS(Cone, "cone")
