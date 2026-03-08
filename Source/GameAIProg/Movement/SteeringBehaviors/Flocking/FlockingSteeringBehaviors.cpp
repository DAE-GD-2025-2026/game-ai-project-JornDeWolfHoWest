#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput steering = {};
    FVector2D averagePosition = FVector2D::ZeroVector;

    // Calculate the average position of the neighbors
    auto& neighbors = pFlock->GetNeighbors();
    int nrOfNeighbors = pFlock->GetNrOfNeighbors();

    for (int i = 0; i < nrOfNeighbors; ++i)
    {
        ASteeringAgent* pNeighbor = neighbors[i];
        if (pNeighbor)
        {
            averagePosition += pNeighbor->GetPosition();
        }
    }

    if (nrOfNeighbors > 0)
    {
        averagePosition /= static_cast<float>(nrOfNeighbors);
    }

    // Seek towards the average position
    FVector2D desiredVelocity = averagePosition - pAgent.GetPosition();
    steering.LinearVelocity = desiredVelocity.GetSafeNormal() * pAgent.GetMaxLinearSpeed();
    return steering;
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput steering = {};
    FVector2D separationForce = FVector2D::ZeroVector;

    // Calculate the separation force based on the neighbors
    auto& neighbors = m_pFlock->GetNeighbors();
    int nrOfNeighbors = m_pFlock->GetNrOfNeighbors();

    for (int i = 0; i < nrOfNeighbors; ++i)
    {
        ASteeringAgent* pNeighbor = neighbors[i];
        if (pNeighbor)
        {
            FVector2D toAgent = pAgent.GetPosition() - pNeighbor->GetPosition();
            float distance = toAgent.Length();
            if (distance > 0)
            {
                separationForce += toAgent / (distance * distance);
            }
        }
    }

    steering.LinearVelocity = separationForce.GetSafeNormal() * pAgent.GetMaxLinearSpeed();
    return steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
    SteeringOutput steering = {};
    FVector2D averageVelocity = FVector2D::ZeroVector;
    // Calculate the average velocity of the neighbors
    auto& neighbors = m_pFlock->GetNeighbors();
    int nrOfNeighbors = m_pFlock->GetNrOfNeighbors();
    if (nrOfNeighbors == 0)
    {
        steering.LinearVelocity = pAgent.GetLinearVelocity();
        return steering;
    }

    for (int i = 0; i < nrOfNeighbors; ++i)
    {
        ASteeringAgent* pNeighbor = neighbors[i];
        if (pNeighbor && pNeighbor != &pAgent)
        {
            averageVelocity += averageVelocity.GetSafeNormal() * pAgent.GetMaxLinearSpeed();
        }
    }
    if (nrOfNeighbors > 0)
    {
        averageVelocity /= static_cast<float>(nrOfNeighbors);
    }
    steering.LinearVelocity = averageVelocity - pAgent.GetLinearVelocity();
    return steering;
}