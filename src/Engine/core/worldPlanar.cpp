//
// Created by York Wu on 2023/10/25.
//

#include <core/worldPlanar.h>

using namespace std;

WorldPlanar::WorldPlanar(float xmin, float xmax, float ymin, float ymax) :
WorldBase(PLANAR_WORLD), xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax)
{}

NeighborList WorldPlanar::ComputeNeighbors(const Vector2D &position, float search_radius, const Agent *queryingAgent) const {
    vector<PhantomAgent> phantoms;
    if (agentKDTree != nullptr)
    {
        // compute neighboring agents
        vector<const Agent*> agents;
        computeNeighboringAgents_Flat(position, search_radius, queryingAgent, agents);

        // efficiently add them to the result
        Vector2D offset(0, 0);
        phantoms.resize(agents.size());
        for (size_t i = 0; i < agents.size(); ++i)
            phantoms[i] = PhantomAgent(agents[i], position, offset);
    }

    // compute neighboring obstacles
    vector<LineSegment2D> obstacles;
    computeNeighboringObstacles_Flat(position, search_radius, obstacles);

    return { phantoms, obstacles };
}

