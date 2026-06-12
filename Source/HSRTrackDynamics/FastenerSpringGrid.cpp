#include "FastenerSpringGrid.h"
#include "TrackGenerator.h"
#include "HSRTrackDynamics.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Components/StaticMeshComponent.h"

AFastenerSpringGrid::AFastenerSpringGrid()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
}

void AFastenerSpringGrid::BeginPlay()
{
	Super::BeginPlay();
}

void AFastenerSpringGrid::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!bUseUEPhysicsConstraints)
	{
		UpdateCustomSpringForces(DeltaTime);
	}
}

void AFastenerSpringGrid::InitializeFastenerGrid(ATrackGenerator* TrackGen)
{
	if (!TrackGen)
	{
		UE_LOG(LogHSRTrackDynamics, Error, TEXT("InitializeFastenerGrid: TrackGen is null"));
		return;
	}

	for (auto* Constraint : PhysicsConstraints)
	{
		if (Constraint) Constraint->DestroyComponent();
	}
	PhysicsConstraints.Empty();

	for (auto* Mesh : RailSegmentMeshes)
	{
		if (Mesh) Mesh->DestroyComponent();
	}
	RailSegmentMeshes.Empty();

	for (auto* Mesh : SleeperBodyMeshes)
	{
		if (Mesh) Mesh->DestroyComponent();
	}
	SleeperBodyMeshes.Empty();

	FastenerStates.Empty();

	int32 TotalFasteners = TrackGen->GetFastenerCount();
	CachedSpacing = TrackGen->GetTotalFastenerSpacing();

	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Initializing fastener grid: %d fasteners"), TotalFasteners);

	FastenerStates.Reserve(TotalFasteners);

	int32 NumSleepers = TotalFasteners / 2;

	for (int32 i = 0; i < NumSleepers; ++i)
	{
		FString SleeperName = FString::Printf(TEXT("SleeperBody_%d"), i);
		UStaticMeshComponent* SleeperBody = NewObject<UStaticMeshComponent>(this, *SleeperName);
		SleeperBody->SetupAttachment(GetRootComponent());
		SleeperBody->RegisterComponent();
		SleeperBody->SetSimulatePhysics(true);
		SleeperBody->SetMobility(EComponentMobility::Movable);
		SleeperBody->SetMassOverrideInKg(NAME_None, 300.0f);

		FVector SleeperPos = TrackGen->GetFastenerWorldPosition(i * 2);
		SleeperBody->SetWorldLocation(SleeperPos - FVector(0, 0, 0.1f));
		SleeperBodyMeshes.Add(SleeperBody);
	}

	int32 SegmentsPerFastener = 2;
	int32 NumRailSegments = (TotalFasteners + SegmentsPerFastener - 1) / SegmentsPerFastener;

	for (int32 i = 0; i < NumRailSegments; ++i)
	{
		FString SegName = FString::Printf(TEXT("RailSegment_%d"), i);
		UStaticMeshComponent* RailSeg = NewObject<UStaticMeshComponent>(this, *SegName);
		RailSeg->SetupAttachment(GetRootComponent());
		RailSeg->RegisterComponent();
		RailSeg->SetSimulatePhysics(true);
		RailSeg->SetMobility(EComponentMobility::Movable);

		float SegLength = CachedSpacing * SegmentsPerFastener;
		float RailMass = RailMassPerMeter * SegLength;
		RailSeg->SetMassOverrideInKg(NAME_None, RailMass);
		RailSegmentMeshes.Add(RailSeg);
	}

	for (int32 i = 0; i < TotalFasteners; ++i)
	{
		FFastenerNodeState NodeState;
		NodeState.ConstraintIndex = i;
		FastenerStates.Add(NodeState);

		FVector FastenerPos = TrackGen->GetFastenerWorldPosition(i);

		int32 RailSegIdx = i / SegmentsPerFastener;
		int32 SleeperIdx = i / 2;

		if (bUseUEPhysicsConstraints &&
			RailSegmentMeshes.IsValidIndex(RailSegIdx) &&
			SleeperBodyMeshes.IsValidIndex(SleeperIdx))
		{
			CreatePhysicsConstraint(
				i,
				RailSegmentMeshes[RailSegIdx],
				SleeperBodyMeshes[SleeperIdx],
				FTransform(FastenerPos)
			);
		}

		PositionToFastenerIndex.Add(i, i);
	}

	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Fastener grid initialized: %d constraints, %d rail segments, %d sleepers"),
		PhysicsConstraints.Num(), RailSegmentMeshes.Num(), SleeperBodyMeshes.Num());
}

void AFastenerSpringGrid::CreatePhysicsConstraint(
	int32 Index,
	UStaticMeshComponent* RailSeg,
	UStaticMeshComponent* SleeperBody,
	const FTransform& FastenerWorldTransform)
{
	FString ConstraintName = FString::Printf(TEXT("FastenerConstraint_%d"), Index);
	UPhysicsConstraintComponent* Constraint = NewObject<UPhysicsConstraintComponent>(this, *ConstraintName);
	Constraint->SetupAttachment(GetRootComponent());
	Constraint->RegisterComponent();

	Constraint->SetConstrainedComponents(RailSeg, NAME_None, SleeperBody, NAME_None);

	Constraint->SetLinearXLimit(ELinearConstraintMotion::Limited, 0.005f);
	Constraint->SetLinearYLimit(ELinearConstraintMotion::Limited, 0.005f);
	Constraint->SetLinearZLimit(ELinearConstraintMotion::Limited, 0.010f);

	Constraint->SetAngularSwing1Limit(EAngularConstraintMotion::Limited, 1.0f);
	Constraint->SetAngularSwing2Limit(EAngularConstraintMotion::Limited, 1.0f);
	Constraint->SetAngularTwistLimit(EAngularConstraintMotion::Limited, 1.0f);

	FConstraintInstance ConstraintProfile;

	ConstraintProfile.LinearDriveXDrive.bEnablePositionDrive = true;
	ConstraintProfile.LinearDriveXDrive.bEnableVelocityDrive = true;
	ConstraintProfile.LinearDriveXDrive.Stiffness = StiffnessParams.LongitudinalStiffness;
	ConstraintProfile.LinearDriveXDrive.Damping = StiffnessParams.LongitudinalDamping;
	ConstraintProfile.LinearDriveXDrive.MaxForce = 1.0e8f;

	ConstraintProfile.LinearDriveYDrive.bEnablePositionDrive = true;
	ConstraintProfile.LinearDriveYDrive.bEnableVelocityDrive = true;
	ConstraintProfile.LinearDriveYDrive.Stiffness = StiffnessParams.LateralStiffness;
	ConstraintProfile.LinearDriveYDrive.Damping = StiffnessParams.LateralDamping;
	ConstraintProfile.LinearDriveYDrive.MaxForce = 1.0e8f;

	ConstraintProfile.LinearDriveZDrive.bEnablePositionDrive = true;
	ConstraintProfile.LinearDriveZDrive.bEnableVelocityDrive = true;
	ConstraintProfile.LinearDriveZDrive.Stiffness = StiffnessParams.VerticalStiffness;
	ConstraintProfile.LinearDriveZDrive.Damping = StiffnessParams.VerticalDamping;
	ConstraintProfile.LinearDriveZDrive.MaxForce = 1.0e8f;

	ConstraintProfile.AngularDrive.SwingDrive.bEnablePositionDrive = true;
	ConstraintProfile.AngularDrive.SwingDrive.bEnableVelocityDrive = true;
	ConstraintProfile.AngularDrive.SwingDrive.Stiffness = StiffnessParams.RotationalStiffness;
	ConstraintProfile.AngularDrive.SwingDrive.Damping = StiffnessParams.RotationalDamping;
	ConstraintProfile.AngularDrive.SwingDrive.MaxForce = 1.0e6f;

	ConstraintProfile.AngularDrive.TwistDrive.bEnablePositionDrive = true;
	ConstraintProfile.AngularDrive.TwistDrive.bEnableVelocityDrive = true;
	ConstraintProfile.AngularDrive.TwistDrive.Stiffness = StiffnessParams.RotationalStiffness;
	ConstraintProfile.AngularDrive.TwistDrive.Damping = StiffnessParams.RotationalDamping;
	ConstraintProfile.AngularDrive.TwistDrive.MaxForce = 1.0e6f;

	ConstraintProfile.LinearBreakForce = 5.0e7f;
	ConstraintProfile.AngularBreakTorque = 5.0e5f;

	Constraint->ConstraintInstance = ConstraintProfile;
	Constraint->SetWorldTransform(FastenerWorldTransform);

	PhysicsConstraints.Add(Constraint);
}

void AFastenerSpringGrid::ApplyWheelLoadToRail(int32 FastenerIndex, float VerticalForce)
{
	if (!FastenerStates.IsValidIndex(FastenerIndex)) return;

	FastenerStates[FastenerIndex].ConstraintForce.Z += VerticalForce;

	if (RailSegmentMeshes.IsValidIndex(FastenerIndex / 2))
	{
		UStaticMeshComponent* RailSeg = RailSegmentMeshes[FastenerIndex / 2];
		if (RailSeg && RailSeg->IsSimulatingPhysics())
		{
			FVector FastenerPos = FastenerStates[FastenerIndex].RailDisplacement;
			RailSeg->AddForceAtLocation(
				FVector(0.0f, 0.0f, -VerticalForce),
				RailSeg->GetComponentLocation()
			);
		}
	}
}

FVector AFastenerSpringGrid::GetFastenerReactionForce(int32 FastenerIndex) const
{
	if (FastenerStates.IsValidIndex(FastenerIndex))
	{
		return FastenerStates[FastenerIndex].ConstraintForce;
	}
	return FVector::ZeroVector;
}

void AFastenerSpringGrid::UpdateCustomSpringForces(float DeltaTime)
{
	if (DeltaTime < KINDA_SMALL_NUMBER) return;

	for (int32 i = 0; i < FastenerStates.Num(); ++i)
	{
		FFastenerNodeState& State = FastenerStates[i];

		FVector SpringForce;
		SpringForce.X = -StiffnessParams.LongitudinalStiffness * State.RailDisplacement.X
			- StiffnessParams.LongitudinalDamping * State.RailVelocity.X;
		SpringForce.Y = -StiffnessParams.LateralStiffness * State.RailDisplacement.Y
			- StiffnessParams.LateralDamping * State.RailVelocity.Y;
		SpringForce.Z = -StiffnessParams.VerticalStiffness * State.RailDisplacement.Z
			- StiffnessParams.VerticalDamping * State.RailVelocity.Z;

		if (bEnableNonlinearStiffness)
		{
			ComputeNonlinearCorrection(State);
			SpringForce.X *= (1.0f + NonlinearStiffnessCoefficient * FMath::Abs(State.RailDisplacement.X));
			SpringForce.Y *= (1.0f + NonlinearStiffnessCoefficient * FMath::Abs(State.RailDisplacement.Y));
			SpringForce.Z *= (1.0f + NonlinearStiffnessCoefficient * FMath::Abs(State.RailDisplacement.Z));
		}

		State.ConstraintForce = SpringForce;

		float RailSegMass = RailMassPerMeter * CachedSpacing;
		FVector Acceleration = SpringForce / RailSegMass;

		State.RailVelocity += Acceleration * DeltaTime;
		State.RailDisplacement += State.RailVelocity * DeltaTime;

		float IntegrationDamping = 0.999f;
		State.RailVelocity *= IntegrationDamping;
	}
}

void AFastenerSpringGrid::ComputeNonlinearCorrection(FFastenerNodeState& NodeState)
{
	float DispMag = NodeState.RailDisplacement.Size();
	if (DispMag > 0.002f)
	{
		float HardeningFactor = 1.0f + NonlinearStiffnessCoefficient * (DispMag - 0.002f) * 1000.0f;
		NodeState.ConstraintForce *= HardeningFactor;
	}
}

int32 AFastenerSpringGrid::FindNearestFastenerIndex(const FVector& WorldPosition) const
{
	int32 BestIdx = 0;
	float BestDistSq = MAX_FLT;

	for (int32 i = 0; i < FastenerStates.Num(); ++i)
	{
		float DistSq = 0.0f;
		int32* FoundIdx = PositionToFastenerIndex.Find(i);
		if (!FoundIdx) continue;

		float ApproxDist = FMath::Abs(WorldPosition.X - i * CachedSpacing);
		float DSq = ApproxDist * ApproxDist;
		if (DSq < BestDistSq)
		{
			BestDistSq = DSq;
			BestIdx = i;
		}
	}

	return BestIdx;
}

FVector AFastenerSpringGrid::GetRailDisplacementAtPosition(const FVector& WorldPosition) const
{
	FVector OutDisp;
	InterpolateRailDisplacement(WorldPosition, OutDisp);
	return OutDisp;
}

void AFastenerSpringGrid::InterpolateRailDisplacement(const FVector& WorldPosition, FVector& OutDisplacement) const
{
	if (FastenerStates.Num() == 0)
	{
		OutDisplacement = FVector::ZeroVector;
		return;
	}

	int32 NearestIdx = FindNearestFastenerIndex(WorldPosition);

	if (NearestIdx == 0 || NearestIdx >= FastenerStates.Num() - 1)
	{
		OutDisplacement = FastenerStates[NearestIdx].RailDisplacement;
		return;
	}

	float Fraction = 0.5f;

	FVector D0 = FastenerStates[NearestIdx].RailDisplacement;
	FVector D1 = FastenerStates[NearestIdx + 1].RailDisplacement;
	OutDisplacement = FMath::Lerp(D0, D1, Fraction);
}
