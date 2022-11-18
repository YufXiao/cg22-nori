/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/integrator.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/scene.h>
#include <nori/photon.h>

NORI_NAMESPACE_BEGIN

class PhotonMapper : public Integrator {
public:
    /// Photon map data structure
    typedef PointKDTree<Photon> PhotonMap;

    PhotonMapper(const PropertyList &props) {
        /* Lookup parameters */
        m_photonCount  = props.getInteger("photonCount", 1000000);
        m_photonRadius = props.getFloat("photonRadius", 0.0f /* Default: automatic */);
    }

    virtual void preprocess(const Scene *scene) override {
        cout << "Gathering " << m_photonCount << " photons .. ";
        cout.flush();

        /* Create a sample generator for the preprocess step */
        Sampler *sampler = static_cast<Sampler *>(
            NoriObjectFactory::createInstance("independent", PropertyList()));

        /* Allocate memory for the photon map */
        m_photonMap = std::unique_ptr<PhotonMap>(new PhotonMap());
        m_photonMap->reserve(m_photonCount);

		/* Estimate a default photon radius */
		if (m_photonRadius == 0)
			m_photonRadius = scene->getBoundingBox().getExtents().norm() / 500.0f;

		/* How to add a photon?
		 * m_photonMap->push_back(Photon(
		 *	Point3f(0, 0, 0),  // Position
		 *	Vector3f(0, 0, 1), // Direction
		 *	Color3f(1, 2, 3)   // Power
		 * ));
		 */

		// put your code to trace photons here

		/* Build the photon map */
		int depositedPhotonsCount = 0;
		while (depositedPhotonsCount < m_photonCount) {

            m_emittedCount++;

            Ray3f pathRay;
            Intersection its;

            auto randomEmitter = scene->getRandomEmitter(sampler->next1D());
            Color3f W = randomEmitter->samplePhoton(pathRay, sampler->next2D(), sampler->next2D()) *
                     scene->getLights().size();

            while (true) {
                if (!scene->rayIntersect(pathRay, its)) {
                    break;
                }

                if (its.mesh->getBSDF()->isDiffuse()) {
                    m_photonMap->push_back(Photon(its.p, -pathRay.d, W));
                    depositedPhotonsCount++;
                }

                // russian roulette
                auto p = std::min(W.maxCoeff(), .99f);
                if (sampler->next1D() > p) {
                    break;
                }
                W /= p;

                BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d));
                bRec.uv = its.uv;
                auto bsdfCosThetaOverPdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
                W *= bsdfCosThetaOverPdf;

                pathRay = Ray3f(its.p, its.shFrame.toWorld(bRec.wo));
            }
        }
        m_photonMap->build();
    }

    virtual Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &_ray) const override {
    	
		/* How to find photons?
		 * std::vector<uint32_t> results;
		 * m_photonMap->search(Point3f(0, 0, 0), // lookup position
		 *                     m_photonRadius,   // search radius
		 *                     results);
		 *
		 * for (uint32_t i : results) {
		 *    const Photon &photon = (*m_photonMap)[i];
		 *    cout << "Found photon!" << endl;
		 *    cout << " Position  : " << photon.getPosition().toString() << endl;
		 *    cout << " Power     : " << photon.getPower().toString() << endl;
		 *    cout << " Direction : " << photon.getDirection().toString() << endl;
		 * }
		 */

		// put your code for path tracing with photon gathering here
        Ray3f pathRay = _ray;
        Intersection its;

        Color3f t(1.f);
        Color3f Li(0.f);

        while (true) {

            if (!scene->rayIntersect(pathRay, its)) {
                break;
            }

            if (its.mesh->isEmitter()) {
                EmitterQueryRecord eRec(pathRay.o, its.p, its.shFrame.n);
                Li += t * its.mesh->getEmitter()->eval(eRec);
            }

            if (its.mesh->getBSDF()->isDiffuse()) {
                Color3f photonDensityEstimation(0);
                std::vector<uint32_t> results;
                m_photonMap->search(its.p, m_photonRadius,results);
                for (auto i : results) {
                    const Photon &photon = (*m_photonMap)[i];
                    BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d), its.shFrame.toLocal(photon.getDirection()), ESolidAngle);
                    bRec.uv = its.uv;
                    auto fr = its.mesh->getBSDF()->eval(bRec);
                    photonDensityEstimation += fr * photon.getPower();
                }
                photonDensityEstimation /= M_PI * pow(m_photonRadius, 2) * m_emittedCount;

                Li += t * photonDensityEstimation;
                break;
            }

            // russian roulette with success probability p
            auto p = std::min(t.maxCoeff(), .99f);
            if (sampler->next1D() > p) {
                break;
            }
            t /= p;


            // Sample from BSDF
            BSDFQueryRecord bRec(its.shFrame.toLocal(-pathRay.d));
            bRec.uv = its.uv;
            auto bsdfCosThetaOverPdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            t *= bsdfCosThetaOverPdf;

            pathRay = Ray3f(its.p, its.shFrame.toWorld(bRec.wo));
        }
		return Li;
    }

    virtual std::string toString() const override {
        return tfm::format(
            "PhotonMapper[\n"
            "  photonCount = %i,\n"
            "  photonRadius = %f\n"
            "]",
            m_photonCount,
            m_photonRadius
        );
    }
private:
    /* 
     * Important: m_photonCount is the total number of photons deposited in the photon map,
     * NOT the number of emitted photons. You will need to keep track of those yourself.
     */ 
    int m_photonCount;
    int m_emittedCount;
    float m_photonRadius;
    std::unique_ptr<PhotonMap> m_photonMap;
};

NORI_REGISTER_CLASS(PhotonMapper, "photonmapper");
NORI_NAMESPACE_END
