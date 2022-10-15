#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectIntegrator : public Integrator {
public:
    DirectIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its)){
            return Color3f(0.0f);
        }

        Vector2f sample;
        Color3f color(0.0f);

        for (auto light : scene->getLights()) {
            EmitterQueryRecord lRec;
            lRec.ref = its.p;
            Color3f value = light->sample(lRec, sample);

            if (scene->rayIntersect(lRec.shadowRay)) {
                continue;
            }
            // transform to local
            auto localLRec = its.shFrame.toLocal(lRec.wi);
            auto localRay = its.shFrame.toLocal(-ray.d);
            auto cosineTerm = Frame::cosTheta(localLRec);

            BSDFQueryRecord bsdfRec(localLRec, localRay, ESolidAngle);
            bsdfRec.uv = its.uv;
            auto bsdf = its.mesh->getBSDF()->eval(bsdfRec);

            color += value * cosineTerm * bsdf;
        }
        return color;
    }

    std::string toString() const {
        return "DirectIntegrator[]";
    }

};

NORI_REGISTER_CLASS(DirectIntegrator, "direct");
NORI_NAMESPACE_END