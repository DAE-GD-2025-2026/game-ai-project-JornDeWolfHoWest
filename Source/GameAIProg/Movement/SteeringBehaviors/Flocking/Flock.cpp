#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "VectorTypes.h"
#include "Shared/ImGuiHelpers.h"

#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"

Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	Agents.SetNum(FlockSize);

 // TODO: initialize the flock and the memory pool

	pSeparationBehavior = std::make_unique< Separation>(this);
	pCohesionBehavior = std::make_unique< Cohesion>(this);
	pVelMatchBehavior = std::make_unique< VelocityMatch>(this);

	// Create the agents
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pEvadeBehavior = std::make_unique<Evade>(500.f);
	pBlendedSteering = std::make_unique<BlendedSteering>(
		std::vector<BlendedSteering::WeightedBehavior>{	
			{pSeparationBehavior.get(), 0.7f},
			{pVelMatchBehavior.get(), 0.4f},
			{pCohesionBehavior.get(), 0.5f},
			{pWanderBehavior.get(), 1.f},
			{pEvadeBehavior.get(), 1.f}
		}
	);
	
	Agents.Reserve(FlockSize);
	for (int i = 0; i < FlockSize; ++i)
	{
		Agents[i] = pWorld->SpawnActor<ASteeringAgent>(AgentClass, FVector{ FMath::RandRange(0.f, WorldSize), FMath::RandRange(0.f, WorldSize),90}, FRotator::ZeroRotator);
		if (Agents[i])
		{
			Agents[i]->SetSteeringBehavior(pBlendedSteering.get());
			Agents[i]->SetIsAutoOrienting(true);
			Agents[i]->SetMaxLinearSpeed(500);
		}
		else
			--i;
	}

	pPrioritySteering = std::make_unique<PrioritySteering>(
		std::vector<ISteeringBehavior*>{ pSeekBehavior.get(), pWanderBehavior.get() }
	);

	pAgentToEvade->SetSteeringBehavior(pPrioritySteering.get());
	/*m_pAgentToEvade->SetSteeringBehavior(new BlendedSteering(
		{
			{m_pWanderBehavior, 1.f}
		}));*/
	pAgentToEvade->SetMaxLinearSpeed(500.f);
	pAgentToEvade->SetIsAutoOrienting(true);
}

Flock::~Flock()
{
 // TODO: Cleanup any additional data
}

void Flock::Tick(float DeltaTime)
{
	pAgentToEvade->Tick(DeltaTime);	
 // TODO: update the flock
 // TODO: for every agent:
  // TODO: register the neighbors for this agent (-> fill the memory pool with the neighbors for the currently evaluated agent)
  // TODO: update the agent (-> the steeringbehaviors use the neighbors in the memory pool)
  // TODO: trim the agent to the world
	pEvadeBehavior->SetTarget(FSteeringParams(pAgentToEvade->GetPosition()));
	for (ASteeringAgent* pAgent : Agents)
	{
		// Register the neighbors for this agentm_pAgentToEvade
		RegisterNeighbors(pAgent);

		// Update the agent using its steering behavior
		pAgent->Tick(DeltaTime);

	}
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();
		
		ImGui::SliderFloat("Seperation", &pBlendedSteering->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f, "%.1f");
		ImGui::SliderFloat("Velocity match", &pBlendedSteering->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f, "%.1f");
		ImGui::SliderFloat("Cohesion", &pBlendedSteering->GetWeightedBehaviorsRef()[2].Weight, 0.f, 1.f, "%.1f");
		ImGui::SliderFloat("Wander", &pBlendedSteering->GetWeightedBehaviorsRef()[3].Weight, 0.f, 1.f, "%.1f");
		ImGui::SliderFloat("Evade", &pBlendedSteering->GetWeightedBehaviorsRef()[4].Weight, 0.f, 1.f, "%.1f");

  // TODO: implement ImGUI sliders for steering behavior weights here
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
 // TODO: Debugrender the neighbors for the first agent in the flock
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	Neighbors.Reset(); // Reset the memory pool
	NrOfNeighbors = 0; // Reset the count

	for (ASteeringAgent* pOtherAgent : Agents)
	{
		if (pOtherAgent == pAgent)
			continue;

		float distanceSquared = UE::Geometry::Distance(pAgent->GetPosition(), pOtherAgent->GetPosition());
		if (distanceSquared < NeighborhoodRadius * NeighborhoodRadius)
		{
			Neighbors.Push(pOtherAgent);
			++NrOfNeighbors;
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	FVector2D avgPosition = FVector2D::ZeroVector;

 // TODO: Implement
	
	return avgPosition;
}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	FVector2D avgVelocity = FVector2D::ZeroVector;

 // TODO: Implement

	return avgVelocity;
}

void Flock::SetTarget_Seek(FSteeringParams const& Target)
{
 // TODO: Implement
	pSeekBehavior->SetTarget(Target);
}

