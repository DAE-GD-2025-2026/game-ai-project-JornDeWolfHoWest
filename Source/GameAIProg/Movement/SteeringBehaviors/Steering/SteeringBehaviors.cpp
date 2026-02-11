#include "SteeringBehaviors.h"

#include "VectorTypes.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& agent)
{
	SteeringOutput steering{};

	steering.LinearVelocity = Target.Position - agent.GetPosition();

	// Add debug rendering
	// TODO
	if (agent.GetDebugRenderingEnabled())
	{
		auto lineEnd{agent.GetPosition() + steering.LinearVelocity.GetSafeNormal() * 100};
		DrawDebugLine(agent.GetWorld(), agent.GetActorLocation(), FVector{lineEnd.X, lineEnd.Y, agent.GetActorLocation().Z}, FColor::Green, false, -1, 0, 5);
	}

	return steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& agent)
{	
	SteeringOutput steering{Seek::CalculateSteering(DeltaT, agent)};
	steering.LinearVelocity *= -1;
	
	return steering;
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& agent)
{
	SteeringOutput steering{Seek::CalculateSteering(DeltaT, agent)};

	// Get distance between agent position & target
	float distanceToTarget = (Target.Position - agent.GetPosition()).Length();
	// Substract the target circle
	distanceToTarget -= TargetRadius;
	// Lerp
	if (distanceToTarget > 0)
		agent.SetMaxLinearSpeed(
			FMath::Lerp(0, 600, FMath::Clamp(distanceToTarget / (SlowRadius - TargetRadius), 0.15f, 1.f)));
	else
		agent.SetMaxLinearSpeed(0);

	if (agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(agent.GetWorld(), FVector(Target.Position.X, Target.Position.Y, 0), 50, 10, FColor::Red, false, -1.f, 0, 4.f, FVector::XAxisVector, FVector::YAxisVector);
		DrawDebugCircle(agent.GetWorld(), agent.GetActorLocation(), TargetRadius, 10, FColor::Red, false, -1.f, 0, 4.f, FVector::XAxisVector, FVector::YAxisVector);
		DrawDebugCircle(agent.GetWorld(), agent.GetActorLocation(), SlowRadius, 10, FColor::Green, false, -1.f, 0, 4.f, FVector::XAxisVector, FVector::YAxisVector);
	}

	return steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& agent)
{
	return SteeringOutput{};
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& agent)
{
	// Get the distance
	float distanceToTarget = (Target.Position - agent.GetPosition()).Length();
	// Calc time to target
	float timeToTarget = distanceToTarget / agent.GetMaxLinearSpeed();
	// Calculate speed of target
	float targetSpeed = (Target.Position - lastTargetPosition).Length();
	// Calculate where target will be in that time
	lastTargetPosition = Target.Position;
	Target.Position += (Target.LinearVelocity) * timeToTarget;

	// Get the distance to the new target
	float distanceToWhereTargetWillBe = (Target.Position - agent.GetPosition()).Length();
	if (distanceToWhereTargetWillBe < 30)
	{
		Target.Position = lastTargetPosition;
	}
	if (agent.GetDebugRenderingEnabled())
	{
		DrawDebugCircle(agent.GetWorld(), FVector(Target.Position.X, Target.Position.Y, 0), 20, 10, FColor::Orange,
		                false, -1.f, 0, 4.f, FVector::XAxisVector, FVector::YAxisVector);
	}

	return Seek::CalculateSteering(DeltaT, agent);
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& agent)
{
	return SteeringOutput{};
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& agent)
{
	
	return Seek::CalculateSteering(DeltaT, agent);
}
