#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator {
public:
    PathMisIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
        Color3f Li(0.f);
        Color3f t(1.f);

        Ray3f pathRay = ray;
        float w_mat = 1.f;
        float w_em = 1.f;

        Intersection its;
        if (!scene->rayIntersect(pathRay, its))
            return Li;

        while (true) {
            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord lRec(pathRay.o, its.p, its.shFrame.n);
                Li += t * w_mat * its.mesh->getEmitter()->eval(lRec);
            }

            const Emitter *emitter = scene->getRandomEmitter(sampler->next1D());
            EmitterQueryRecord lRec(its.p);
            Color3f LeOverPdf = emitter->sample(lRec, sampler->next2D()) * scene->getLights().size();
            float pdf_em = emitter->pdf(lRec);
            if (!scene->rayIntersect(lRec.shadowRay)) 
            {
                float cosTheta = std::max(0.f, Frame::cosTheta(its.shFrame.toLocal(lRec.wi)));
                BSDFQueryRecord bRec(its.toLocal(-pathRay.d), its.toLocal(lRec.wi), ESolidAngle);
                bRec.uv = its.uv;
                Color3f f = its.mesh->getBSDF()->eval(bRec);
                float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

                if (pdf_mat + pdf_em > 0.f) {
                    w_em = pdf_em / (pdf_mat + pdf_em);
                }
                else w_em = pdf_em;
                Li += LeOverPdf * f * cosTheta * w_em * t;
            }

            //Russian roulettio
            auto p = std::min(t.maxCoeff(), .99f);
            if (sampler->next1D() > p)  break;
            t /= p;

            BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d));
            bRec.uv = its.uv;
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            t *= f;

            pathRay = Ray3f(its.p, its.toWorld(bRec.wo));

            float pdf_mat = its.mesh->getBSDF()->pdf(bRec);

            Point3f origin = its.p;
            if (!scene->rayIntersect(pathRay, its))
                return Li;

            if (its.mesh->isEmitter()) {
                EmitterQueryRecord lRec = EmitterQueryRecord(origin, its.p, its.shFrame.n);
                float pdf_em = its.mesh->getEmitter()->pdf(lRec);
                if (pdf_mat + pdf_em > 0.f) {
                    w_mat = pdf_mat / (pdf_mat + pdf_em);
                }
                else w_mat = pdf_mat;
            }

            if (bRec.measure == EDiscrete)
                w_mat = 1.0f;
        }
        return Li;
    }

    std::string toString() const {
        return "PathMisIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END
