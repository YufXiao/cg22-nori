#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

class PointLight : public Emitter
{
private:
    Color3f m_power;
    Point3f m_position;

public:
    PointLight(const PropertyList &props) {
        m_position = props.getPoint3("position");
        m_power = props.getColor("power");
    }

    Color3f sample(EmitterQueryRecord &lRec, const Point2f &sample) const override {
        lRec.p = m_position;
        auto direction = (lRec.p - lRec.ref);
        lRec.wi = direction.normalized();
        lRec.pdf = pdf(lRec);
        lRec.shadowRay = Ray3f(lRec.ref, lRec.wi, Epsilon, direction.norm() - Epsilon);
        return eval(lRec);
    }

    Color3f eval(const EmitterQueryRecord &lRec) const override {
        return m_power / (4 * M_PI * (lRec.p - lRec.ref).squaredNorm());
    }

    float pdf(const EmitterQueryRecord &lRec) const override {
        return 1.f;
    }

    std::string toString() const override {
        return "PointLight[]";
    }
};

NORI_REGISTER_CLASS(PointLight, "point");
NORI_NAMESPACE_END