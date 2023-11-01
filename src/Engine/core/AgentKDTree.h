/* UMANS: Unified Microscopic Agent Navigation Simulator
** MIT License
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettr�
**
** Permission is hereby granted, free of charge, to any person obtaining
** a copy of this software and associated documentation files (the
** "Software"), to deal in the Software without restriction, including
** without limitation the rights to use, copy, modify, merge, publish,
** distribute, sublicense, and/or sell copies of the Software, and to
** permit persons to whom the Software is furnished to do so, subject
** to the following conditions:
**
** The above copyright notice and this permission notice shall be
** included in all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
** EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
** OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
** NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
** LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
** ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
** CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
**
** Contact: crowd_group@inria.fr
** Website: https://project.inria.fr/crowdscience/
** See the file AUTHORS.md for a list of all contributors.
*/

#ifndef LIB_AGENTKDTREE_H
#define LIB_AGENTKDTREE_H

#include <vector>
#include <core/agent.h>
#include <3rd-party/nanoflann/nanoflann.hpp>

/// <summary>A 2-dimensional KD-tree of agent positions (using the *nanoflann* library), which can be used for nearest-neighbor queries.</summary>
/// <remarks>Note: The KD-tree does not automatically change over time as the agents move.
/// Therefore, the simulation must create a new AgentKDTree in each frame.</remarks>
class AgentKDTree
{
private:

	/// <summary>A point-cloud wrapper for agent positions, required for the *nanoflann* library.</summary>
	struct AgentPointCloud
	{
		std::vector<std::pair<size_t, Vector2D>> agentPositions;

		AgentPointCloud(const std::vector<Agent*>& agents)
		{
			agentPositions.resize(agents.size());
			for (size_t i = 0; i < agents.size(); ++i)
				agentPositions[i] = std::pair<size_t, Vector2D>(agents[i]->getID(), agents[i]->getPosition());
		}

		// Must return the number of data points
		inline size_t kdtree_get_point_count() const { return agentPositions.size(); }

		// Returns the dim'th component of the idx'th point in the class:
		// Since this is inlined and the "dim" argument is typically an immediate value, the
		//  "if/else's" are actually solved at compile time.
		inline double kdtree_get_pt(const size_t idx, const size_t dim) const
		{
			if (dim == 0) return agentPositions[idx].second.x;
			else return agentPositions[idx].second.y;
		}

		// Optional bounding-box computation: return false to default to a standard bbox computation loop.
		//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
		//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
		template <class BBOX>
		bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
	};

	typedef nanoflann::KDTreeSingleIndexAdaptor<
		nanoflann::L2_Simple_Adaptor<double, AgentPointCloud>,
		AgentPointCloud, 2> NanoflannKDTree;

	AgentPointCloud pointCloud;
	NanoflannKDTree* kdTree;

public:
	/// <summary>Creates an AgentKDTree for a given list of agents, using the *current* position of these agents.</summary>
	/// <param name="agents">A list of agents.</param>
	AgentKDTree(const std::vector<Agent*>& agents);

	/// <summary>Cleans up this AgentKDTree for removal.</summary>
	~AgentKDTree();

	/// <summary>Computes and returns the IDs of all agents that lie within a given radius of a given position.</summary>
	/// <param name="position">A query position.</param>
	/// <param name="radius">A query radius.</param>
	/// <param name="agentToIgnore">A pointer to the Agent object that should be excluded from the results.
	/// This is most likely the agent for which the query is being performed.
	/// Use nullptr to not exclude any agents.</param>
	/// <returns>A list of IDs of agents that lie within "radius" meters of "position", 
	/// excluding the ID of the agent denoted by "agentToIgnore" (if it exists).</returns>
	std::vector<size_t> FindAllAgentsInRange(const Vector2D& position, const float radius, const Agent* agentToIgnore) const;

	/// <summary>Computes and returns the *k* nearest agents to a given position.</summary>
	/// <param name="position">A query position.</param>
	/// <param name="k">The maximum number of neighbors to retrieve.</param>
	/// <param name="agentToIgnore">A pointer to the Agent object that should be excluded from the results.
	/// This is most likely the agent for which the query is being performed.
	/// Use nullptr to not exclude any agents.</param>
	/// <returns>A list of IDs of the "k" agents that are closest to "position", 
	/// excluding the ID of the agent denoted by "agentToIgnore" (if it exists). 
	/// Note: The list may have less than "k" entries if there are not enough agents in the world.</returns>
	std::vector<size_t> FindKNearestAgents(const Vector2D& position, const size_t k, const Agent* agentToIgnore) const;
};

#endif //LIB_AGENTKDTREE_H