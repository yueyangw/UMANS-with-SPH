//
// Created by York Wu on 2023/10/25.
//

#ifndef UMANS_WORLDPLANAR_H
#define UMANS_WORLDPLANAR_H

#include <core/worldBase.h>

class WorldPlanar : public WorldBase {
private:
    float xmin_;
    float xmax_;
    float ymin_;
    float ymax_;

public:
    WorldPlanar(float xmin, float xmax, float ymin, float ymax);

    inline float getXmin() const {
        return xmin_;
    }

    inline float getXmax() const {
        return xmax_;
    }

    inline float getYmin() const {
        return ymin_;
    }

    inline float getYmax() const {
        return ymax_;
    }

    NeighborList ComputeNeighbors(const Vector2D &position, float search_radius, const Agent *queryingAgent) const override;
};

#endif //UMANS_WORLDPLANAR_H
