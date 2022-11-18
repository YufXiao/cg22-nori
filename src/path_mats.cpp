#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class PathMatsIntegrator : public Integrator {
public:
    PathMatsIntegrator(const PropertyList& props) {}

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
        Color3f Li(0.f);
        Color3f t(1.f);

        Intersection its;
        Ray3f pathRay = ray;

        while (true)
        {
            bool sceneIntersected = scene->rayIntersect(pathRay, its);

            if (sceneIntersected) {
                Color3f Le(0.f);
                if (its.mesh->isEmitter()) {
                    EmitterQueryRecord lRec(pathRay.o, its.p, its.shFrame.n);
                    Le = its.mesh->getEmitter()->eval(lRec);
                }
                
                Li += t * Le;
                // russian roulette
                auto p = std::min(t.maxCoeff(), .99f);
                if (sampler->next1D() > p)  break;
                t /= p;
                BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d));
                
                bRec.uv = its.uv;
                auto frCosThetaOverPdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
                t *= frCosThetaOverPdf;
                pathRay = Ray3f(its.p, its.shFrame.toWorld(bRec.wo));
            }
            else break;
        }
        return Li;
    }
    std::string toString() const override {
        return "PathMatsIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMatsIntegrator, "path_mats");
NORI_NAMESPACE_END