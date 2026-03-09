#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>
#include "Kismet/KismetMathLibrary.h"

class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...


class Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual ~Seek() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& agent);
};

class Flee : public Seek
{
public:
	Flee() = default;
	virtual ~Flee() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& agent);
};

class Arrive : public Seek
{
public:
	Arrive() = default;
	virtual ~Arrive() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& agent);
	void SetTargetRadius(float radius) { TargetRadius = radius; };
private:
	float TargetRadius{30};
	float SlowRadius{230};
};

class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& agent);
};

class Pursuit : public Seek
{
public:
	Pursuit() = default;
	virtual ~Pursuit() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& agent);
private:
	FVector2D lastTargetPosition;
};

class Evade : public Pursuit
{
public:
	Evade() : EvasionRadius(500.f) {};
	Evade(const float evasionDistance) : EvasionRadius(evasionDistance) {};
	virtual ~Evade() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& agent);
	const float EvasionRadius;
};
class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() = default;

	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent& agent);
	
	void SetWanderOffset(float offset) { m_OffsetDistance = offset; }
	void SetWanderRadius(float radius) { m_Radius = radius; }
	void SetMaxAngleChange(float rad) { m_MaxAngleChange = rad; }
private:
	float m_OffsetDistance{ 150.f };
	float m_Radius{ 4.f };
	float m_MaxAngleChange{ FMath::DegreesToRadians(45)};
	float m_WanderAngle{ 0.f };
};