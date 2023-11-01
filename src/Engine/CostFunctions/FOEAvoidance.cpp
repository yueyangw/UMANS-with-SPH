/* UMANS: Unified Microscopic Agent Navigation Simulator
** MIT License
** Copyright (C) 2018-2020  Inria Rennes Bretagne Atlantique - Rainbow - Julien Pettré
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

#include <CostFunctions/FOEAvoidance.h>
#include <core/agent.h>
#include <core/worldBase.h>
#include <tools/Matrix.h>
#include <tools/HelperFunctions.h>

using namespace std;

constexpr auto SMALL_NUMBER = 0.01;
constexpr auto MIN_DISTANCE = 0.1;
constexpr auto MIN_VELOCITY = 0.01;

float sgn(float val)
{
	return (float)((0 < val) - (val < 0));
}

float Importance(float ttc)
{
	if (ttc < 0.f || ttc > 10.f)
		return 0.f;

	float t0 = 5, I0 = 10, a = -I0 / t0;
	float I = a * ttc + I0;
	return HelperFunctions::Clamp<float>(I, 0.1f, 10.f);
	//return 1;
}

float FOEAvoidance::GetCost(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	if (velocity.isZero())
		return 0; 
	
	const Vector2D& Position = agent->getPosition();
	const Vector2D& Vnorm = velocity.getnormalized();
	Matrix R(Vector2D(Vnorm.y, -Vnorm.x), Vnorm);
	const Matrix& RT = R.GetTransposed();
	const float rangeSquared = range_ * range_;
	const auto& neighbors = agent->getNeighbors();

	float Cost = 0;

	// check neighboring agents
	for (const PhantomAgent& neighbor : neighbors.first)
	{
		if (neighbor.GetDistanceSquared() > rangeSquared)
			continue;

		Vector2D Vel = RT * (neighbor.GetVelocity() - velocity);
		Vector2D Pos = RT * (neighbor.GetPosition() - Position);

		if (Pos.y < MIN_DISTANCE || Vel.y > -MIN_VELOCITY)
			continue;

		float xf = Vel.x / Vel.y;
		float xg = Pos.x / Pos.y;
		float dx = xg - xf;
		float ttc = -Pos.y / Vel.y;

		float sigma = (float)(((agent->getRadius() + neighbor.realAgent->getRadius()) / Pos.y) / log(10));

		float localCost = Importance(ttc)*exp(-abs(dx) / sigma);

		Cost += localCost;
	}

	// TODO: check neighboring obstacles
	// ...

	return Cost;
}

Vector2D FOEAvoidance::GetGradient(const Vector2D& velocity, Agent* agent, const WorldBase * world) const
{
	if (velocity.isZero())
		return Vector2D(0,0); 

	const Vector2D& Position = agent->getPosition();
	const Vector2D& Vnorm = velocity.getnormalized();
	Matrix R(Vector2D(Vnorm.y, -Vnorm.x), Vnorm);
	const Matrix& RT = R.GetTransposed();
	const float rangeSquared = range_ * range_;
	const auto& neighbors = agent->getNeighbors();

	float gradtheta = 0, gradv = 0;

	// check neighboring agents
	for (const PhantomAgent& neighbor : neighbors.first)
	{
		if (neighbor.GetDistanceSquared() > rangeSquared)
			continue;

		Vector2D Vel = RT * (neighbor.GetVelocity() - velocity);
		Vector2D Pos = RT * (neighbor.GetPosition() - Position);

		if (Pos.y < MIN_DISTANCE || Vel.y > -MIN_VELOCITY)
			continue;

		float xf = Vel.x / Vel.y;
		float xg = Pos.x / Pos.y;
		float dx = xg - xf;
		float ttc = -Pos.y / Vel.y;

		float sigma = (float)(((agent->getRadius() + neighbor.realAgent->getRadius()) / Pos.y) / log(10));

		float localgradtheta = Importance(ttc)*exp(-abs(dx) / sigma) * sgn(dx)*(xf*xf - xg * xg - (velocity.magnitude() / (-Vel.y))) / sigma;
		float localgradv = Importance(ttc)*exp(-abs(dx) / sigma) * sgn(dx)*(-xf / (-Vel.y)) / sigma;

		gradtheta += localgradtheta;
		gradv += localgradv;
	}

	// TODO: check neighboring obstacles
	// ...

	gradtheta = HelperFunctions::Clamp(gradtheta, -0.1f, 0.1f);
	//gradtheta = -gradtheta;
	//gradv = HelperFunctions::Clamp(gradv, -1.f, 1.f);
	//gradv = 0;

	return -1 * R*(velocity.magnitude()*Vector2D(sin(gradtheta), 1 - cos(gradtheta)) + Vector2D(0, gradv));
}