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

#include <core/worldBase.h>
#include <omp.h>

using namespace nanoflann;
using namespace std;

WorldBase::Type WorldBase::StringToWorldType(const std::string& type)
{
    if (type == "Infinite")
		return Type::INFINITE_WORLD;
    else if (type == "Toric")
		return Type::TORIC_WORLD;
    else if (type == "Planar")
        return Type::PLANAR_WORLD;
    else
		return Type::UNKNOWN_WORLD_TYPE;
}

WorldBase::WorldBase(WorldBase::Type type) : type_(type)
{
	time_ = 0;
	agentKDTree = nullptr;
	SetNumberOfThreads(1);
	nextUnusedAgentID = 0;
}

void WorldBase::SetNumberOfThreads(int nrThreads)
{
	omp_set_num_threads(nrThreads);
}

void WorldBase::computeNeighboringAgents_Flat(const Vector2D& position, float search_radius, const Agent* queryingAgent, std::vector<const Agent*>& result) const
{
	// get the IDs of neighbors
	const auto& agentIDs = agentKDTree->FindAllAgentsInRange(position, search_radius, queryingAgent);

	// convert them to agent pointers
	result.resize(agentIDs.size());
	for (int i = 0; i < agentIDs.size(); ++i)
		result[i] = GetAgent_const(agentIDs[i]);
}

void WorldBase::computeNeighboringObstacles_Flat(const Vector2D& position, float search_radius, std::vector<LineSegment2D>& result) const
{
	// TODO: implement more efficient neighbor search for obstacles
	const float radiusSquared = search_radius * search_radius;

	// loop over all obstacle edges
	for (const auto& obs : obstacles_)
	{
		for (const auto& edge : obs.GetEdges())
		{
			// if this line segment is clsoe enough, add it to the result
			if (distanceToLineSquared(position, edge.first, edge.second, true) <= radiusSquared)
				result.push_back(edge);
		}
	}
}

void WorldBase::DoStep()
{
	// Before the simulation frame begins, add agents that need to be added now
	while (!agentsToAdd.empty() && agentsToAdd.top().second <= time_)
	{
		addAgentToList(agentsToAdd.top().first);
		agentsToAdd.pop();
	}
	
	// --- Main simulation tasks:
	// 1. build the KD tree for nearest-neighbor computations
	if (agentKDTree != nullptr)
		delete agentKDTree;
	agentKDTree = new AgentKDTree(agents_);

	int n = (int)agents_.size();

// These missions processed in ComputeAcceleration
    // 2. compute nearest neighbors for each agent
// #pragma omp parallel for
//     for (int i = 0; i < n; ++i)
// 		agents_[i]->ComputeNeighbors(this);

// 	// 3. compute a preferred velocity for each agent
// #pragma omp parallel for
// 	for (int i = 0; i < n; ++i)
// 		agents_[i]->ComputePreferredVelocity();

    // TODO:吴越洋1030添加，计算SPH密度
#pragma omp parallel for
    for (int i = 1; i < n; ++i)
        agents_[i]->ComputeSPHDensity(this);

	// 4. perform local navigation for each agent, to compute an acceleration vector for them
#pragma omp parallel for
	for (int i = 0; i < n; ++i)
		agents_[i]->ComputeAcceleration(this);

	// 5. compute contact forces for all agents
#pragma omp parallel for
	for (int i = 0; i < n; ++i)
		agents_[i]->ComputeContactForces(this);

	// 6. move all agents to their new positions
	DoStep_MoveAllAgents();
	
	// --- End of main simulation tasks.

	// increase the time that has passed
	time_ += delta_time_;

	// remove agents who have reached their goal
	for (int i = n-1; i >= 0; --i)
		if (agents_[i]->getRemoveAtGoal() && agents_[i]->hasReachedGoal())
			removeAgentAtListIndex(i);
}

void WorldBase::DoStep_MoveAllAgents()
{
#pragma omp parallel for
	for (int i = 0; i < (int)agents_.size(); i++)
		agents_[i]->UpdateVelocityAndPosition(this);
}

#pragma region [Finding, adding, and removing agents]

Agent* WorldBase::GetAgent(size_t id)
{
	// find out if the agent with this ID exists
	auto positionInList = agentPositionsInVector.find(id);
	if (positionInList == agentPositionsInVector.end())
		return nullptr;

	// return the agent who's at the correct position in the list
	return agents_[positionInList->second];
}

const Agent* WorldBase::GetAgent_const(size_t id) const
{
	// find out if the agent with this ID exists
	auto positionInList = agentPositionsInVector.find(id);
	if (positionInList == agentPositionsInVector.end())
		return nullptr;

	// return the agent who's at the correct position in the list
	return agents_[positionInList->second];
}

Agent* WorldBase::AddAgent(const Vector2D& position, const Agent::Settings& settings, size_t desiredID, float startTime)
{
	// determine the right ID for the agent
	size_t agentID;
	if (desiredID != std::numeric_limits<size_t>::max() && GetAgent(desiredID) == nullptr) // custom ID specified by the user; use it only if it's not already taken
		agentID = desiredID;
	else // determine a good ID automatically
		agentID = nextUnusedAgentID;

	// create the agent and set its position
	Agent* agent = new Agent(agentID, settings);
	agent->setPosition(position);

	// if the new ID is the highest one so far, update the next available ID
	if (agentID >= nextUnusedAgentID)
		nextUnusedAgentID = agentID + 1;

	// if desired, add the agent immediately
	if (startTime <= time_)
		addAgentToList(agent);

	// otherwise, schedule the agent for insertion at a later time
	else
		agentsToAdd.push({ agent, startTime });

	return agent;
}

void WorldBase::addAgentToList(Agent* agent)
{
	// add the agent to the list, and store where in the list it is located
	agentPositionsInVector[agent->getID()] = agents_.size();
	agents_.push_back(agent);
}

bool WorldBase::RemoveAgent(size_t id)
{
	// find out if the agent with this ID exists
	auto positionInList = agentPositionsInVector.find(id);
	if (positionInList == agentPositionsInVector.end())
		return false;

	removeAgentAtListIndex(positionInList->second);
	return true;
}

void WorldBase::removeAgentAtListIndex(size_t index)
{
	const auto removedAgentID = agents_[index]->getID();
	
	// if the agent is at the end of the list, simply remove it
	if (index + 1 == agents_.size())
	{
		agents_.pop_back();
		agentPositionsInVector.erase(removedAgentID);
	}
	
	// if the agent is not at the end of the list, do some extra management
	else
	{
		// delete the requested agent
		delete agents_[index];

		// move the last agent in the list to the position that has now become free
		auto lastAgent = agents_.back();
		agents_[index] = lastAgent;
		agents_.pop_back();

		// update the position map:
		// - the requested agent is now gone
		agentPositionsInVector.erase(removedAgentID);
		// - another agent has moved
		agentPositionsInVector[lastAgent->getID()] = index;
	}
}

#pragma endregion

void WorldBase::AddObstacle(const std::vector<Vector2D>& points)
{
	obstacles_.push_back(Polygon2D(points));
}

WorldBase::~WorldBase()
{
	// delete the KD tree
	if (agentKDTree != nullptr)
		delete agentKDTree;

	// delete all agents
	for (Agent* agent : agents_)
		delete agent;
	agents_.clear();

	// delete all agents that were scheduled for insertion
	while (!agentsToAdd.empty())
	{
		delete agentsToAdd.top().first;
		agentsToAdd.pop();
	}

	// delete the mapping from IDs to agents
	agentPositionsInVector.clear();
	nextUnusedAgentID = 0;
}