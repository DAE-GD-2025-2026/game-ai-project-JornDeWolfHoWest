#pragma once
#include "Movement/SteeringBehaviors/Steering/SteeringBehaviors.h"
class Flock;

//COHESION - FLOCKING
//*******************
class Cohesion final : public Seek
{
public:
	Cohesion(Flock* const pFlock) :pFlock(pFlock) {};

	//Cohesion Behavior
	SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
	Flock* pFlock = nullptr;
};

//SEPARATION - FLOCKING
//*********************
class Separation : public ISteeringBehavior
{
public:
    Separation(Flock* const pFlock) :m_pFlock(pFlock) {};

    //Separation Behavior
    SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
    Flock* m_pFlock = nullptr;
};

//VELOCITY MATCH - FLOCKING
//************************
class VelocityMatch : public ISteeringBehavior
{
public:
    VelocityMatch(Flock* const pFlock) :m_pFlock(pFlock) {};

    //Velocity Match Behavior
    SteeringOutput CalculateSteering(float deltaT, ASteeringAgent& pAgent) override;

private:
    Flock* m_pFlock = nullptr;
};
