/* UMANS: Unified Microscopic Agent Navigation Simulator
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettr�
**
** This program is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program. If not, see <https://www.gnu.org/licenses/>.
**
** Contact: crowd_group@inria.fr
** Website: https://project.inria.fr/crowdscience/
** See the file AUTHORS.md for a list of all contributors.
*/

#include <CostFunctions/SPH.h>
#include <core/agent.h>
#include <core/worldBase.h>
#include <tools/HelperFunctions.h>

const bool UseObstacleParticles_Density = false;
const bool UseObstacleParticles_PressureForce = false;

void SPH::ComputeDensityData(const Agent* agent, const float dt, SPH::DensityData& result, SPH::DensityData& result_progressive) const
{
	const auto& neighbors = agent->getNeighbors();
	const Vector2D& agentPos = agent->getPosition();

	result.density = 0;
	result.pressure = 0;

	// compute density: sum up the density contribution of all neighbors
	
	// - add neighboring agents
	for (const auto& neighbor : neighbors.first)
	{
		if (!UseObstacleParticles_Density && neighbor.realAgent->isSPHObstacleParticle())
			continue;

		float diff = rangeSquared - neighbor.GetDistanceSquared();
		if (diff > 0)
			result.density += neighbor.realAgent->getMass() * POLY_6 * powf(diff, 3.0f);
	}

	// - add the agent itself
	result.density += agent->getMass() * baseDensityContribution;

	// - add neighboring obstacles (but not if this agent itself is a boundary particle)
	if (!UseObstacleParticles_Density && !agent->isSPHObstacleParticle())
	{
		for (const auto& neighborObs : neighbors.second)
		{
			// compute the area V that this obstacle segment occupies inside the kernel circle
			float obsVolume = getObstacleVolumeInsideCircle(neighborObs, agentPos, range_);
			if (obsVolume > 0)
			{
				float distanceToObstacle = distanceToLine(agentPos, neighborObs.first, neighborObs.second, true);
				float distanceToCenterOfMass = (range_ + distanceToObstacle) / 2.0f;
				// density_b = restDensity * V * W(distance) 
				float diff = rangeSquared - distanceToCenterOfMass*distanceToCenterOfMass;
				result.density += obsVolume * result.restDensity * POLY_6 * powf(diff, 3.0f);
			}
		}
	}

	// update the progressive average density over time
	
	float frac = dt / densityAdaptationTime;
	result_progressive.density = (1 - frac) * result_progressive.density + frac * result.density;

	// compute the current rest density

	result.restDensity = HelperFunctions::Clamp<float>(result_progressive.density, restDensityMin, restDensityMax);

	// compute pressure
	result.pressure = gasConstant * (result.density - result.restDensity);
}

Vector2D SPH::ComputeAgentInteractionForce(const Agent* agent, const PhantomAgent& other) const
{
	const DensityData& data_i = agent->getSPHDensityData();
	const DensityData& data_j = other.realAgent->getSPHDensityData();
	float mass_i = agent->getMass();
	float mass_j = other.realAgent->getMass();

	Vector2D result(0, 0);

	// if we've chosen to ignore obstacle particles, and this neighbor is an obstacle particle, ignore it
	if (!UseObstacleParticles_PressureForce && other.realAgent->isSPHObstacleParticle())
		return result;

	// if the neighboring agent is too far away, ignore it (this shouldn't happen; distant agents have already been filtered out)
	if (other.GetDistanceSquared() >= rangeSquared)
		return result;

	const float dist = sqrt(other.GetDistanceSquared());
	const float rangeMinDist = range_ - dist;

	// pressure force
	if (data_i.pressure > 0)
		result += mass_j * (data_i.pressure + data_j.pressure) / (2.0f * data_j.density) * SPIKY_GRAD * rangeMinDist * rangeMinDist / dist * (other.GetPosition() - agent->getPosition());
		//result += mass_j * (data_i.pressure / (data_i.density*data_i.density) + data_j.pressure / (data_j.density*data_j.density)) * SPIKY_GRAD * rangeMinDist * rangeMinDist / dist * (other.GetPosition() - agent->getPosition());

	// viscosity force
	if (viscosity > 0)
		result += viscosity * mass_j * VISC_LAP * rangeMinDist / data_j.density * (other.GetVelocity() - agent->getVelocity());

	return result / data_i.density;
}

Vector2D SPH::ComputeObstacleInteractionForce(const Agent* agent, const LineSegment2D& obstacle) const
{
	Vector2D result(0, 0);

	// if we've chosen to use obstacle particles instead, ignore the obstacle itself
	if (UseObstacleParticles_PressureForce)
		return result;

	const Vector2D& agentPos = agent->getPosition();
	const DensityData& data_i = agent->getSPHDensityData();
	float mass_i = agent->getMass();

    const Vector2D& nearest = nearestPointOnLine(agentPos, obstacle.first, obstacle.second, true);

	// compute the area V that this obstacle segment occupies inside the kernel circle
	// TODO: maybe store it, so we don't have to compute it twice?
	float obsVolume = getObstacleVolumeInsideCircle(obstacle, agentPos, range_);
	if (obsVolume > 0)
	{
		float distanceToObstacle = distance(agentPos, nearest);
		float distanceToCenterOfMass = (range_ + distanceToObstacle) / 2.0f;
		float rangeMinDist = range_ - distanceToCenterOfMass;

		// pressure force. Assumptions: obstacle pressure == agent pressure, obstacle density == rest density
		if (data_i.pressure > 0)
			result += obsVolume * data_i.pressure * SPIKY_GRAD * rangeMinDist * rangeMinDist / distanceToObstacle * (nearest - agentPos);
			//result += obsVolume * (data_i.pressure / (data_i.density * data_i.density) + (data_i.pressure / (data_i.restDensity * data_i.restDensity))) * SPIKY_GRAD * rangeMinDist * rangeMinDist / dist * (nearest - agentPos);

		// viscosity force is ignored for now; this would make agents stick to walls
	}

	return result / data_i.density;
}

float SPH::getObstacleVolumeInsideCircle(const LineSegment2D& segment, const Vector2D& circleCenter, const float circleRadius) const
{
	// get the part of the obstacle segment that lies inside the circle
	bool isInsideCircle;
	LineSegment2D part(getObstaclePartInsideCircle(segment, circleCenter, circleRadius, isInsideCircle));
	if (!isInsideCircle)
		return 0;

	// compute the circle section bounded by these two points
	float ang = angle(part.first - circleCenter, part.second - circleCenter);
	float circleSectionArea = circleRadius * circleRadius * ang / 2;

	// compute the triangle bounded by the circle center and these two points
	float triangleArea = 0.5f * distance(part.first, part.second) * distanceToLine(circleCenter, segment.first, segment.second);

	return circleSectionArea - triangleArea;
}

LineSegment2D SPH::getObstaclePartInsideCircle(const LineSegment2D& segment, const Vector2D& circleCenter, const float circleRadius, bool& resultIsValid) const
{
	const Vector2D p(segment.first - circleCenter);
	const float radiusSq = circleRadius * circleRadius;

	const Vector2D v(segment.second - segment.first);

	// To find out at where the line intersects the disk, we must solve the following equation:
	//      dist(pCircle, pObsStart + v2*t) = r
	// =>   || PDiff + v*t ||^2 = r^2
	// =>   V.x^2*t^2 + 2*P.x*V.x*t + P.x^2 + (same for .y) = R^2
	// =>   (V.x*V.x + V.y*V.y) * t^2 + 2*(P.x*V.x + P.y*V.y) * t + (P.x*P.x + P.y*P.y) = R^2
	// =>   (V.V)*t^2 + 2*(P.)*t + (P.P) = R^2

	float t1 = MaxFloat, t2 = MaxFloat;
	const int nrSolutions = SolveQuadraticEquation(v.dot(v), 2 * p.dot(v), p.dot(p) - radiusSq, t1, t2);

	// if there are not 2 solutions, then the obstacle does not intersect the circle (or it only barely touches it),
	// so it does not contribute to any SPH quantities.
	if (nrSolutions != 2)
	{
		resultIsValid = false;
		return LineSegment2D();
	}

	// order the two solutions, and bound them to the endpoints of the initial segment
	float tFirst = std::max(0.f, std::min(t1, t2));
	float tSecond = std::min(1.f, std::max(t1, t2));

	// convert back to points, and return the result
	resultIsValid = true;
	return LineSegment2D(segment.first + tFirst * v, segment.first + tSecond * v);
}

void SPH::parseParameters(const CostFunctionParameters& params)
{
	ObjectInteractionForces::parseParameters(params);
	params.ReadFloat("gasConstant", gasConstant);
	params.ReadFloat("restDensityMin", restDensityMin);
	params.ReadFloat("restDensityMax", restDensityMax);
	params.ReadFloat("densityAdaptationTime", densityAdaptationTime);
	params.ReadFloat("viscosity", viscosity);
	params.ReadBool("isObstacle", isObstacle);

	rangeSquared = range_ * range_;

	//POLY_6 = (float)(315.0 / (64*PI*pow(range_, 9))); // 3D version
	POLY_6 = (float)(4.0 / (PI * pow(range_, 8)));

	//SPIKY_GRAD = (float)(-45.0 / (PI*pow(range_, 6))); // 3D version
	SPIKY_GRAD = (float)(-30.0 / (PI*pow(range_, 5)));

	//VISC_LAP = (float)(45.0 / (PI*pow(range_, 6))); // 3D version
	VISC_LAP = (float)(360.0 / (29 * PI*pow(range_, 5)));

	baseDensityContribution = POLY_6 * powf(rangeSquared, 3.0f);
}