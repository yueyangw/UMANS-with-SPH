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

#ifndef LIB_SPH_H
#define LIB_SPH_H

#include <CostFunctions/ObjectInteractionForces.h>

class SPH : public ObjectInteractionForces
{
private:
	float gasConstant = 100.f;
	float restDensityMin = 0.f;
	float restDensityMax = 5.f;
	float densityAdaptationTime = 0.1f; // number of seconds over which to compute the average density
	float viscosity = 0;
	bool isObstacle = false;

	// Constants used in kernel functions; they can be precomputed as soon as range_ has been set.
	float POLY_6, SPIKY_GRAD, VISC_LAP;

	float rangeSquared;
	float baseDensityContribution;

public:
	SPH(const PolicyStep* step) : ObjectInteractionForces(step, false) { range_ = 1; }
	virtual ~SPH() {}
	const static std::string GetName() { return "SPH"; }
	const bool AssumesObstacleParticles() const { return isObstacle; }

	void parseParameters(const CostFunctionParameters& params) override;

	struct DensityData
	{
		float density;
		float pressure;
		float restDensity;

		DensityData() : density(0), pressure(0), restDensity(0) {}
	};

	void ComputeDensityData(const Agent* agent, float dt, DensityData& result, DensityData& result_progressive) const;

	inline const float GetRestDensityMin() const { return restDensityMin; }
	inline const float GetRestDensityMax() const { return restDensityMax; }
	inline const float GetDensityAdaptationTime() const { return densityAdaptationTime; }

protected:
	virtual Vector2D ComputeAgentInteractionForce(const Agent* agent, const PhantomAgent& other) const override;
	virtual Vector2D ComputeObstacleInteractionForce(const Agent* agent, const LineSegment2D& obstacle, const Vector2D& nearest) const override;

private:
	float getObstacleVolumeInsideCircle(const LineSegment2D& segment, const Vector2D& circleCenter, const float circleRadius) const;
	LineSegment2D getObstaclePartInsideCircle(const LineSegment2D& segment, const Vector2D& circleCenter, const float circleRadius, bool& resultIsValid) const;
};

#endif //LIB_SPH_H
