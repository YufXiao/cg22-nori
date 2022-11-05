#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMisIntegrator : public Integrator {
    public:
        DirectMisIntegrator(const PropertyList &props) {}

        Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const override {
            Intersection its;
            if(!scene->rayIntersect(ray, its)) {
                return Color3f(0.f);
            }

            Color3f Le(0.f);

            if (its.mesh->isEmitter()) {
                EmitterQueryRecord lRec(ray.o, its.p, its.shFrame.n);
                Le = its.mesh->getEmitter()->eval(lRec);
            }

            auto wEm = Color3f(0.f);
            auto wMat = Color3f(0.f);

            // ems part
            {
                for (auto light : scene->getLights()) {
                    EmitterQueryRecord lRec;
                    lRec.ref = its.p;
                    auto LeOverPdf = light->sample(lRec, sampler->next2D());
                    auto pdfEm = light->pdf(lRec);

                    if (scene->rayIntersect(lRec.shadowRay)) {
                        continue;
                    }

                    auto localRay = its.shFrame.toLocal(-ray.d); // incident
                    auto localLRec = its.shFrame.toLocal(lRec.wi); // outgoing

                    auto cosTheta = Frame::cosTheta(localLRec);

                    BSDFQueryRecord bsdfRec(localRay, localLRec, ESolidAngle);
                    bsdfRec.uv = its.uv;
                    auto fr = its.mesh->getBSDF()->eval(bsdfRec);
                    auto pdfMat = its.mesh->getBSDF()->pdf(bsdfRec);

                    auto weight = 0.f;
                    if (pdfEm + pdfMat != 0.f) {
                        weight = pdfEm / (pdfEm + pdfMat);
                    }
                    Color3f F = fr * LeOverPdf * cosTheta;
                    wEm += weight * F;
                }
            }

            // Mats part
            {
                BSDFQueryRecord bRec(its.shFrame.toLocal(-ray.d));
                bRec.uv = its.uv;
                auto frCosineThetaOverPdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
                auto pdfMat = its.mesh->getBSDF()->pdf(bRec);

                Ray3f wi(its.p, its.shFrame.toWorld(bRec.wo));

                Intersection itsWi;
                auto Le = Color3f(0.f);
                auto pdfEm = 0.f;
                if (scene->rayIntersect(wi, itsWi) && itsWi.mesh->isEmitter()) {
                    EmitterQueryRecord lRec(wi.o, itsWi.p, itsWi.shFrame.n);
                    Le = itsWi.mesh->getEmitter()->eval(lRec);
                    pdfEm = itsWi.mesh->getEmitter()->pdf(lRec);
                }

                auto weight = 0.f;
                if (pdfEm + pdfMat != 0.f) {
                    weight = pdfMat / (pdfEm + pdfMat);
                }
                wMat = weight * Le * frCosineThetaOverPdf;
            }

            return Le + wEm + wMat;
        }

        std::string toString() const override {
            return "DirectMisIntegrator[]";
        }
};

NORI_REGISTER_CLASS(DirectMisIntegrator, "direct_mis");
NORI_NAMESPACE_END