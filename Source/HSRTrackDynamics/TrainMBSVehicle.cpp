#include "TrainMBSVehicle.h"
#include "HSRTrackDynamics.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Components/StaticMeshComponent.h"

ATrainMBSVehicle::ATrainMBSVehicle()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;

	CarBodyMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CarBody"));
	RootComponent = CarBodyMesh;
}

void ATrainMBSVehicle::BeginPlay()
{
	Super::BeginPlay();
}

void ATrainMBSVehicle::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	StepMBSDynamics(DeltaTime);
}

void ATrainMBSVehicle::InitializeVehicle()
{
	WheelsetStates.SetNum(NUM_WHEELSETS);
	BogieStates.SetNum(NUM_BOGIES);
	WheelContactResults.SetNum(NUM_WHEELS);

	CreatePhysicsBodies();
	CreateSuspensionConstraints();

	for (int32 i = 0; i < NUM_WHEELS; ++i)
	{
		UWheelRailContact* WRC = NewObject<UWheelRailContact>(this,
			*FString::Printf(TEXT("WheelRailContact_%d"), i));
		WRC->RegisterComponent();
		WRC->HertzParams.WheelRadius = WheelRadius;
		WRC->WheelLoad = AxleLoad / 2.0f;
		WRC->VehicleSpeed = TargetSpeed;
		WheelRailContacts.Add(WRC);
	}

	if (bUseRK4Integration)
	{
		InitializeRK4States();

		if (bDisableChaosPhysics)
		{
			for (auto* Body : WheelsetBodies)
			{
				if (Body) Body->SetSimulatePhysics(false);
			}
			for (auto* Body : BogieFrameBodies)
			{
				if (Body) Body->SetSimulatePhysics(false);
			}
			if (CarBodyMesh)
			{
				CarBodyMesh->SetSimulatePhysics(false);
			}
			UE_LOG(LogHSRTrackDynamics, Log,
				TEXT("Chaos physics disabled - using RK4 custom integrator"));
		}
	}

	if (!DerailmentAssessor)
	{
		DerailmentAssessor = NewObject<UDerailmentAssessor>(this, TEXT("DerailmentAssessor"));
		DerailmentAssessor->RegisterComponent();
		DerailmentAssessor->StaticWheelLoad = AxleLoad / 2.0f;
		DerailmentAssessor->NumWheels = NUM_WHEELS;
	}

	UE_LOG(LogHSRTrackDynamics, Log, TEXT("MBS Vehicle initialized: %d wheelsets, %d bogies, %d wheels"),
		NUM_WHEELSETS, NUM_BOGIES, NUM_WHEELS);
}

void ATrainMBSVehicle::CreatePhysicsBodies()
{
	for (auto* Body : WheelsetBodies)
	{
		if (Body) Body->DestroyComponent();
	}
	WheelsetBodies.Empty();

	for (auto* Body : BogieFrameBodies)
	{
		if (Body) Body->DestroyComponent();
	}
	BogieFrameBodies.Empty();

	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		UStaticMeshComponent* WheelsetBody = NewObject<UStaticMeshComponent>(this,
			*FString::Printf(TEXT("Wheelset_%d"), i));
		WheelsetBody->SetupAttachment(RootComponent);
		WheelsetBody->RegisterComponent();
		WheelsetBody->SetSimulatePhysics(true);
		WheelsetBody->SetMobility(EComponentMobility::Movable);
		WheelsetBody->SetMassOverrideInKg(NAME_None, WheelsetMass);

		int32 BogieIdx = i / NUM_WHEELSETS_PER_BOGIE;
		int32 AxleIdx = i % NUM_WHEELSETS_PER_BOGIE;
		float BogieOffset = (BogieIdx == 0) ? -BogieHalfSpacing : BogieHalfSpacing;
		float AxleOffset = (AxleIdx == 0) ? -AxleHalfSpacing : AxleHalfSpacing;

		WheelsetBody->SetWorldLocation(GetActorLocation() +
			FVector(BogieOffset + AxleOffset, 0.0f, WheelRadius));

		WheelsetBodies.Add(WheelsetBody);
	}

	for (int32 i = 0; i < NUM_BOGIES; ++i)
	{
		UStaticMeshComponent* BogieFrame = NewObject<UStaticMeshComponent>(this,
			*FString::Printf(TEXT("BogieFrame_%d"), i));
		BogieFrame->SetupAttachment(RootComponent);
		BogieFrame->RegisterComponent();
		BogieFrame->SetSimulatePhysics(true);
		BogieFrame->SetMobility(EComponentMobility::Movable);
		BogieFrame->SetMassOverrideInKg(NAME_None, BogieFrameMass);

		float BogieOffset = (i == 0) ? -BogieHalfSpacing : BogieHalfSpacing;
		BogieFrame->SetWorldLocation(GetActorLocation() +
			FVector(BogieOffset, 0.0f, WheelRadius + PrimaryVerticalOffset));

		BogieFrameBodies.Add(BogieFrame);
	}

	if (CarBodyMesh)
	{
		CarBodyMesh->SetSimulatePhysics(true);
		CarBodyMesh->SetMassOverrideInKg(NAME_None, CarBodyMass);
		CarBodyMesh->SetWorldLocation(GetActorLocation() +
			FVector(0.0f, 0.0f, WheelRadius + PrimaryVerticalOffset + SecondaryVerticalOffset));
	}
}

void ATrainMBSVehicle::CreateSuspensionConstraints()
{
	for (auto* C : PrimaryConstraints)
	{
		if (C) C->DestroyComponent();
	}
	PrimaryConstraints.Empty();

	for (auto* C : SecondaryConstraints)
	{
		if (C) C->DestroyComponent();
	}
	SecondaryConstraints.Empty();

	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		int32 BogieIdx = i / NUM_WHEELSETS_PER_BOGIE;

		UPhysicsConstraintComponent* PrimConstraint = NewObject<UPhysicsConstraintComponent>(this,
			*FString::Printf(TEXT("PrimarySusp_%d"), i));
		PrimConstraint->SetupAttachment(RootComponent);
		PrimConstraint->RegisterComponent();

		PrimConstraint->SetConstrainedComponents(
			WheelsetBodies[i], NAME_None,
			BogieFrameBodies[BogieIdx], NAME_None);

		FConstraintInstance Profile;

		Profile.LinearDriveXDrive.bEnablePositionDrive = true;
		Profile.LinearDriveXDrive.bEnableVelocityDrive = true;
		Profile.LinearDriveXDrive.Stiffness = PrimarySuspension.AxleBoxSpringStiffnessX;
		Profile.LinearDriveXDrive.Damping = PrimarySuspension.AxleBoxDampingX;
		Profile.LinearDriveXDrive.MaxForce = 5.0e6f;

		Profile.LinearDriveYDrive.bEnablePositionDrive = true;
		Profile.LinearDriveYDrive.bEnableVelocityDrive = true;
		Profile.LinearDriveYDrive.Stiffness = PrimarySuspension.AxleBoxSpringStiffnessY;
		Profile.LinearDriveYDrive.Damping = PrimarySuspension.AxleBoxDampingY;
		Profile.LinearDriveYDrive.MaxForce = 5.0e6f;

		Profile.LinearDriveZDrive.bEnablePositionDrive = true;
		Profile.LinearDriveZDrive.bEnableVelocityDrive = true;
		Profile.LinearDriveZDrive.Stiffness = PrimarySuspension.AxleBoxSpringStiffnessZ;
		Profile.LinearDriveZDrive.Damping = PrimarySuspension.AxleBoxDampingZ;
		Profile.LinearDriveZDrive.MaxForce = 5.0e6f;

		Profile.AngularDrive.SwingDrive.bEnablePositionDrive = true;
		Profile.AngularDrive.SwingDrive.bEnableVelocityDrive = true;
		Profile.AngularDrive.SwingDrive.Stiffness = PrimarySuspension.PrimaryVerticalStiffnessRoll;
		Profile.AngularDrive.SwingDrive.Damping = PrimarySuspension.PrimaryDampingRoll;

		Profile.SetLinearXLimit(ELinearConstraintMotion::Limited, 0.01f);
		Profile.SetLinearYLimit(ELinearConstraintMotion::Limited, 0.015f);
		Profile.SetLinearZLimit(ELinearConstraintMotion::Limited, 0.02f);
		Profile.SetAngularSwing1Limit(EAngularConstraintMotion::Limited, 2.0f);
		Profile.SetAngularSwing2Limit(EAngularConstraintMotion::Limited, 2.0f);
		Profile.SetAngularTwistLimit(EAngularConstraintMotion::Limited, 1.0f);

		PrimConstraint->ConstraintInstance = Profile;
		PrimaryConstraints.Add(PrimConstraint);
	}

	for (int32 i = 0; i < NUM_BOGIES; ++i)
	{
		UPhysicsConstraintComponent* SecConstraint = NewObject<UPhysicsConstraintComponent>(this,
			*FString::Printf(TEXT("SecondarySusp_%d"), i));
		SecConstraint->SetupAttachment(RootComponent);
		SecConstraint->RegisterComponent();

		SecConstraint->SetConstrainedComponents(
			BogieFrameBodies[i], NAME_None,
			CarBodyMesh, NAME_None);

		FConstraintInstance Profile;

		Profile.LinearDriveXDrive.bEnablePositionDrive = true;
		Profile.LinearDriveXDrive.bEnableVelocityDrive = true;
		Profile.LinearDriveXDrive.Stiffness = SecondarySuspension.AirSpringStiffnessX;
		Profile.LinearDriveXDrive.Damping = SecondarySuspension.AirSpringDampingX;
		Profile.LinearDriveXDrive.MaxForce = 3.0e6f;

		Profile.LinearDriveYDrive.bEnablePositionDrive = true;
		Profile.LinearDriveYDrive.bEnableVelocityDrive = true;
		Profile.LinearDriveYDrive.Stiffness = SecondarySuspension.AirSpringStiffnessY;
		Profile.LinearDriveYDrive.Damping = SecondarySuspension.AirSpringDampingY;
		Profile.LinearDriveYDrive.MaxForce = 3.0e6f;

		Profile.LinearDriveZDrive.bEnablePositionDrive = true;
		Profile.LinearDriveZDrive.bEnableVelocityDrive = true;
		Profile.LinearDriveZDrive.Stiffness = SecondarySuspension.AirSpringStiffnessZ;
		Profile.LinearDriveZDrive.Damping = SecondarySuspension.AirSpringDampingZ;
		Profile.LinearDriveZDrive.MaxForce = 3.0e6f;

		Profile.AngularDrive.SwingDrive.bEnablePositionDrive = true;
		Profile.AngularDrive.SwingDrive.bEnableVelocityDrive = true;
		Profile.AngularDrive.SwingDrive.Stiffness = SecondarySuspension.SecondaryRollStiffness;
		Profile.AngularDrive.SwingDrive.Damping = SecondarySuspension.SecondaryRollDamping;

		Profile.SetLinearXLimit(ELinearConstraintMotion::Limited, 0.02f);
		Profile.SetLinearYLimit(ELinearConstraintMotion::Limited, 0.03f);
		Profile.SetLinearZLimit(ELinearConstraintMotion::Limited, 0.03f);
		Profile.SetAngularSwing1Limit(EAngularConstraintMotion::Limited, 3.0f);
		Profile.SetAngularSwing2Limit(EAngularConstraintMotion::Limited, 3.0f);
		Profile.SetAngularTwistLimit(EAngularConstraintMotion::Limited, 2.0f);

		SecConstraint->ConstraintInstance = Profile;
		SecondaryConstraints.Add(SecConstraint);
	}
}

void ATrainMBSVehicle::StepMBSDynamics(float DeltaTime)
{
	if (DeltaTime < KINDA_SMALL_NUMBER) return;

	if (bUseRK4Integration)
	{
		StepRK4Dynamics(DeltaTime);
		return;
	}

	ComputeWheelRailForces();

	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		FVector PrimForce, PrimTorque;
		ComputePrimarySuspensionForces(i, PrimForce, PrimTorque);
		IntegrateWheelsetState(i, DeltaTime);
	}

	for (int32 i = 0; i < NUM_BOGIES; ++i)
	{
		FVector SecForce, SecTorque;
		ComputeSecondarySuspensionForces(i, SecForce, SecTorque);
		IntegrateBogieState(i, DeltaTime);
	}

	IntegrateCarBodyState(DeltaTime);

	if (bEnableTractionControl)
	{
		ApplyTractionControl(DeltaTime);
	}

	if (CarBodyMesh && CarBodyMesh->IsSimulatingPhysics())
	{
		CarBodyMesh->SetWorldLocation(GetActorLocation() +
			FVector(CarBodyState.LongitudinalVel > 0 ? 0.01f : 0, 0, 0));
	}
}

void ATrainMBSVehicle::ComputeWheelRailForces()
{
	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		FWheelsetDofState& WS = WheelsetStates[i];

		for (int32 side = 0; side < 2; ++side)
		{
			int32 WheelIdx = i * 2 + side;
			bool bLeft = (side == 0);

			if (WheelRailContacts.IsValidIndex(WheelIdx))
			{
				UWheelRailContact* WRC = WheelRailContacts[WheelIdx];
				if (!WRC) continue;

				WRC->WheelCenterPosition = GetWheelContactPosition(i, bLeft) +
					FVector(0, 0, WheelRadius);
				WRC->WheelVelocity = FVector(WS.LongitudinalVel, WS.LateralVel, WS.VerticalVel);
				WRC->WheelAngularVelocity = WS.WheelAngularVel;
				WRC->WheelLateralDisplacement = WS.LateralDisp;
				WRC->VehicleSpeed = FMath::Abs(WS.LongitudinalVel);

				float EffectiveRadius = WheelRadius - 0.05f * FMath::Abs(WS.LateralDisp);
				float LateralOffset = bLeft ? -Gauge / 2.0f : Gauge / 2.0f;
				WRC->RailTopPosition = GetActorLocation() +
					FVector(WS.LongitudinalPos, LateralOffset + WS.LateralDisp, 0);

				WheelContactResults[WheelIdx] = WRC->LastContactResult;
			}
		}
	}
}

void ATrainMBSVehicle::ComputePrimarySuspensionForces(int32 WheelsetIdx, FVector& OutForce, FVector& OutTorque)
{
	if (!WheelsetStates.IsValidIndex(WheelsetIdx)) return;

	FWheelsetDofState& WS = WheelsetStates[WheelsetIdx];
	int32 BogieIdx = WheelsetIdx / NUM_WHEELSETS_PER_BOGIE;
	FBogieDofState& BS = BogieStates[BogieIdx];

	float RelLateral = WS.LateralDisp - BS.LateralDisp;
	float RelVertical = WS.VerticalDisp - BS.VerticalDisp;
	float RelRoll = WS.RollAngle - BS.RollAngle;
	float RelYaw = WS.YawAngle - BS.YawAngle;

	float RelLatVel = WS.LateralVel - BS.LateralVel;
	float RelVertVel = WS.VerticalVel - BS.VerticalVel;
	float RelRollRate = WS.RollRate - BS.RollRate;
	float RelYawRate = WS.YawRate - BS.YawRate;

	OutForce.X = -PrimarySuspension.AxleBoxSpringStiffnessX * WS.LongitudinalVel
		- PrimarySuspension.AxleBoxDampingX * WS.LongitudinalVel;
	OutForce.Y = -PrimarySuspension.AxleBoxSpringStiffnessY * RelLateral
		- PrimarySuspension.AxleBoxDampingY * RelLatVel;
	OutForce.Z = -PrimarySuspension.AxleBoxSpringStiffnessZ * RelVertical
		- PrimarySuspension.AxleBoxDampingZ * RelVertVel;

	OutForce.Y += ComputeLateralBumperForce(RelLateral, RelLatVel).Y;

	OutTorque.X = -PrimarySuspension.PrimaryVerticalStiffnessRoll * RelRoll
		- PrimarySuspension.PrimaryDampingRoll * RelRollRate;
	OutTorque.Z = -PrimarySuspension.AxleBoxSpringStiffnessY * AxleHalfSpacing * AxleHalfSpacing * RelYaw
		- PrimarySuspension.AxleBoxDampingY * AxleHalfSpacing * AxleHalfSpacing * RelYawRate;

	for (int32 side = 0; side < 2; ++side)
	{
		int32 WheelIdx = WheelsetIdx * 2 + side;
		if (WheelContactResults.IsValidIndex(WheelIdx))
		{
			OutForce += WheelContactResults[WheelIdx].TangentialForce;
			OutForce.Z += WheelContactResults[WheelIdx].NormalForce;

			if (WheelsetBodies.IsValidIndex(WheelsetIdx) && WheelsetBodies[WheelsetIdx])
			{
				WheelsetBodies[WheelsetIdx]->AddForce(OutForce);
				WheelsetBodies[WheelsetIdx]->AddTorque(OutTorque);
			}
		}
	}
}

void ATrainMBSVehicle::ComputeSecondarySuspensionForces(int32 BogieIdx, FVector& OutForce, FVector& OutTorque)
{
	if (!BogieStates.IsValidIndex(BogieIdx)) return;

	FBogieDofState& BS = BogieStates[BogieIdx];
	FCarBodyDofState& CS = CarBodyState;

	float RelLateral = BS.LateralDisp - CS.LateralDisp;
	float RelVertical = BS.VerticalDisp - CS.VerticalDisp;
	float RelRoll = BS.RollAngle - CS.RollAngle;
	float RelPitch = BS.PitchAngle - CS.PitchAngle;
	float RelYaw = BS.YawAngle - CS.YawAngle;

	float RelLatVel = BS.LateralVel - CS.LateralVel;
	float RelVertVel = BS.VerticalVel - CS.VerticalVel;
	float RelRollRate = BS.RollRate - CS.RollRate;
	float RelPitchRate = BS.PitchRate - CS.PitchRate;
	float RelYawRate = BS.YawRate - CS.YawRate;

	float AirSpringForceZ = ComputeAirSpringForce(RelVertical, RelVertVel, BogieIdx);

	OutForce.X = -SecondarySuspension.AirSpringStiffnessX * BS.LateralDisp
		- SecondarySuspension.AirSpringDampingX * BS.LateralVel;
	OutForce.Y = -SecondarySuspension.AirSpringStiffnessY * RelLateral
		- SecondarySuspension.AirSpringDampingY * RelLatVel;
	OutForce.Z = AirSpringForceZ;

	OutTorque.X = -SecondarySuspension.SecondaryRollStiffness * RelRoll
		- SecondarySuspension.SecondaryRollDamping * RelRollRate;
	OutTorque.Y = -SecondarySuspension.AirSpringStiffnessX * BogieHalfSpacing * RelPitch
		- SecondarySuspension.AirSpringDampingX * BogieHalfSpacing * RelPitchRate;
	OutTorque.Z = -SecondarySuspension.AirSpringStiffnessY * BogieHalfSpacing * RelYaw
		- SecondarySuspension.AirSpringDampingY * BogieHalfSpacing * RelYawRate;

	for (int32 ws = 0; ws < NUM_WHEELSETS_PER_BOGIE; ++ws)
	{
		int32 WSIdx = BogieIdx * NUM_WHEELSETS_PER_BOGIE + ws;
		FVector PrimForce, PrimTorque;
		ComputePrimarySuspensionForces(WSIdx, PrimForce, PrimTorque);
		OutForce -= PrimForce;
		OutTorque -= PrimTorque;
	}

	if (BogieFrameBodies.IsValidIndex(BogieIdx) && BogieFrameBodies[BogieIdx])
	{
		BogieFrameBodies[BogieIdx]->AddForce(OutForce);
		BogieFrameBodies[BogieIdx]->AddTorque(OutTorque);
	}
}

float ATrainMBSVehicle::ComputeAirSpringForce(float Displacement, float Velocity, int32 BogieIdx)
{
	float LinearForce = -SecondarySuspension.AirSpringStiffnessZ * Displacement
		- SecondarySuspension.AirSpringDampingZ * Velocity;

	if (SecondarySuspension.bNonlinearAirSpring)
	{
		float V0 = SecondarySuspension.AirSpringAuxVolume;
		float Aeff = SecondarySuspension.AirSpringEffectiveArea;
		float P0 = SecondarySuspension.AirSpringInitialPressure;
		float N = SecondarySuspension.AirSpringPolytropicIndex;

		float CurrentHeight = V0 / Aeff;
		float NewHeight = CurrentHeight - Displacement;
		if (FMath::Abs(NewHeight) < KINDA_SMALL_NUMBER) NewHeight = KINDA_SMALL_NUMBER;

		float CurrentVolume = Aeff * CurrentHeight;
		float NewVolume = Aeff * NewHeight;

		if (NewVolume < KINDA_SMALL_NUMBER) NewVolume = KINDA_SMALL_NUMBER;

		float P_Current = P0 * FMath::Pow(CurrentVolume / NewVolume, N);
		float NonlinearForce = (P_Current - P0) * Aeff;

		return NonlinearForce - SecondarySuspension.AirSpringDampingZ * Velocity;
	}

	return LinearForce;
}

void ATrainMBSVehicle::IntegrateWheelsetState(int32 Idx, float Dt)
{
	if (!WheelsetStates.IsValidIndex(Idx)) return;

	FWheelsetDofState& WS = WheelsetStates[Idx];

	float AxZ = -WS.VerticalVel * 10.0f / WheelsetMass;
	float AyZ = -WS.LateralVel * 10.0f / WheelsetMass;

	WS.VerticalVel += AxZ * Dt;
	WS.VerticalDisp += WS.VerticalVel * Dt;

	WS.LateralVel += AyZ * Dt;
	WS.LateralDisp += WS.LateralVel * Dt;

	WS.LongitudinalVel = TargetSpeed;
	WS.LongitudinalPos += WS.LongitudinalVel * Dt;

	float Conicity = 0.05f;
	float GravitationalStiffness = WheelsetMass * 9.81f * Conicity / WheelRadius;
	WS.YawRate = -WS.LateralDisp * GravitationalStiffness / (WheelsetInertiaZZ * 0.1f);
	WS.YawAngle += WS.YawRate * Dt;

	WS.RollAngle = Conicity * WS.LateralDisp / WheelRadius;
	WS.RollRate = Conicity * WS.LateralVel / WheelRadius;

	float RollingOmega = WS.LongitudinalVel / WheelRadius;
	WS.WheelAngularVel = RollingOmega;
}

void ATrainMBSVehicle::IntegrateBogieState(int32 Idx, float Dt)
{
	if (!BogieStates.IsValidIndex(Idx)) return;

	FBogieDofState& BS = BogieStates[Idx];

	BS.VerticalVel *= 0.99f;
	BS.VerticalDisp += BS.VerticalVel * Dt;

	BS.LateralVel *= 0.99f;
	BS.LateralDisp += BS.LateralVel * Dt;

	BS.YawRate *= 0.98f;
	BS.YawAngle += BS.YawRate * Dt;

	BS.RollRate *= 0.99f;
	BS.RollAngle += BS.RollRate * Dt;

	BS.PitchRate *= 0.99f;
	BS.PitchAngle += BS.PitchRate * Dt;
}

void ATrainMBSVehicle::IntegrateCarBodyState(float Dt)
{
	FCarBodyDofState& CS = CarBodyState;

	CS.LongitudinalVel = TargetSpeed;

	CS.VerticalVel *= 0.999f;
	CS.VerticalDisp += CS.VerticalVel * Dt;

	CS.LateralVel *= 0.999f;
	CS.LateralDisp += CS.LateralVel * Dt;

	CS.YawRate *= 0.999f;
	CS.YawAngle += CS.YawRate * Dt;

	CS.RollRate *= 0.999f;
	CS.RollAngle += CS.RollRate * Dt;

	CS.PitchRate *= 0.999f;
	CS.PitchAngle += CS.PitchRate * Dt;
}

FVector ATrainMBSVehicle::ComputeLateralBumperForce(float LateralDisp, float LateralVel) const
{
	FVector BumperForce = FVector::ZeroVector;

	float AbsDisp = FMath::Abs(LateralDisp);
	if (AbsDisp > SecondarySuspension.LateralBumperGap)
	{
		float Penetration = AbsDisp - SecondarySuspension.LateralBumperGap;
		float Sign = FMath::Sign(LateralDisp);
		BumperForce.Y = -Sign * SecondarySuspension.LateralBumperStiffness * Penetration
			- SecondarySuspension.LateralBumperStiffness * 0.1f * LateralVel;
	}

	return BumperForce;
}

void ATrainMBSVehicle::ApplyTractionControl(float DeltaTime)
{
	float SpeedError = TargetSpeed - CarBodyState.LongitudinalVel;
	SpeedErrorIntegral += SpeedError * DeltaTime;
	SpeedErrorIntegral = FMath::Clamp(SpeedErrorIntegral, -1.0f, 1.0f);

	float TractionForce = TractionGain * SpeedError + TractionDerivativeGain * SpeedError / DeltaTime;
	TractionForce = FMath::Clamp(TractionForce, -5.0e5f, 5.0e5f);

	if (CarBodyMesh && CarBodyMesh->IsSimulatingPhysics())
	{
		FVector ForceDir = CarBodyMesh->GetForwardVector();
		CarBodyMesh->AddForce(ForceDir * TractionForce);
	}

	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		if (WheelsetBodies.IsValidIndex(i) && WheelsetBodies[i] && WheelsetBodies[i]->IsSimulatingPhysics())
		{
			WheelsetBodies[i]->AddForce(
				WheelsetBodies[i]->GetForwardVector() * TractionForce / NUM_WHEELSETS);
		}
	}
}

FVector ATrainMBSVehicle::GetWheelContactPosition(int32 WheelsetIdx, bool bLeftWheel) const
{
	if (!WheelsetStates.IsValidIndex(WheelsetIdx)) return FVector::ZeroVector;

	const FWheelsetDofState& WS = WheelsetStates[WheelsetIdx];
	int32 BogieIdx = WheelsetIdx / NUM_WHEELSETS_PER_BOGIE;
	int32 AxleIdx = WheelsetIdx % NUM_WHEELSETS_PER_BOGIE;

	float BogieOffset = (BogieIdx == 0) ? -BogieHalfSpacing : BogieHalfSpacing;
	float AxleOffset = (AxleIdx == 0) ? -AxleHalfSpacing : AxleHalfSpacing;
	float LateralOffset = bLeftWheel ? -Gauge / 2.0f : Gauge / 2.0f;

	return GetActorLocation() + FVector(
		BogieOffset + AxleOffset + WS.LongitudinalPos,
		LateralOffset + WS.LateralDisp,
		WS.VerticalDisp
	);
}

float ATrainMBSVehicle::GetCurrentSpeed() const
{
	return bUseRK4Integration ? RK4CarBodyState.Velocity.X : CarBodyState.LongitudinalVel;
}

void ATrainMBSVehicle::InitializeRK4States()
{
	RK4WheelsetStates.SetNum(NUM_WHEELSETS);
	RK4CarBodyState = FRK4CarBodyState();

	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		int32 BogieIdx = i / NUM_WHEELSETS_PER_BOGIE;
		int32 AxleIdx = i % NUM_WHEELSETS_PER_BOGIE;
		float BogieOffset = (BogieIdx == 0) ? -BogieHalfSpacing : BogieHalfSpacing;
		float AxleOffset = (AxleIdx == 0) ? -AxleHalfSpacing : AxleHalfSpacing;

		RK4WheelsetStates[i].Position = FVector(
			BogieOffset + AxleOffset,
			0.0f,
			WheelRadius + PrimaryVerticalOffset
		);
		RK4WheelsetStates[i].Velocity = FVector(TargetSpeed, 0.0f, 0.0f);
		RK4WheelsetStates[i].WheelSpinVelocity = TargetSpeed / WheelRadius;
		RK4WheelsetStates[i].Rotation = FQuat::Identity;
		RK4WheelsetStates[i].AngularVelocity = FVector::ZeroVector;
	}

	RK4CarBodyState.Position = FVector(0.0f, 0.0f, WheelRadius + PrimaryVerticalOffset + SecondaryVerticalOffset);
	RK4CarBodyState.Velocity = FVector(TargetSpeed, 0.0f, 0.0f);
	RK4CarBodyState.Rotation = FQuat::Identity;
	RK4CarBodyState.AngularVelocity = FVector::ZeroVector;

	if (!NumericalStabilizer)
	{
		NumericalStabilizer = NewObject<UNumericalStabilizer>(this, TEXT("NumericalStabilizer"));
		NumericalStabilizer->RegisterComponent();
	}

	TotalMechanicalEnergy = ComputeTotalMechanicalEnergy();
	VelocityClampCount = 0;
	EnergyCorrectionCount = 0;
	CurrentStiffnessScale = 1.0f;
	CurrentDampingScale = 1.0f;

	float NaturalFreq = FMath::Sqrt(PrimarySuspension.AxleBoxSpringStiffnessZ / WheelsetMass);
	DominantFrequency = NaturalFreq / (2.0f * PI);

	UE_LOG(LogHSRTrackDynamics, Log,
		TEXT("RK4 States initialized: %d wheelsets, dominant freq=%.1f Hz, base substep=%.1e s"),
		NUM_WHEELSETS, DominantFrequency, RK4BaseSubstep);
}

void ATrainMBSVehicle::StepRK4Dynamics(float DeltaTime)
{
	if (DeltaTime < KINDA_SMALL_NUMBER) return;

	TotalSubstepsThisFrame = 0;
	float RemainingTime = DeltaTime;
	float CurrentDt = RK4BaseSubstep;
	float LocalTime = 0.0f;

	float EnergyBefore = ComputeTotalMechanicalEnergy();

	while (RemainingTime > KINDA_SMALL_NUMBER)
	{
		float SubDt = FMath::Min(CurrentDt, RemainingTime);

		if (bAdaptiveSubstepping)
		{
			float MinAdaptiveDt = RK4MinSubstep;
			for (int32 i = 0; i < NUM_WHEELSETS; ++i)
			{
				float AdaptiveDt = ComputeAdaptiveSubstepSize(CurrentDt, RK4WheelsetStates[i]);
				MinAdaptiveDt = FMath::Min(MinAdaptiveDt, AdaptiveDt);
			}
			SubDt = FMath::Clamp(MinAdaptiveDt, RK4MinSubstep, RK4BaseSubstep);
			SubDt = FMath::Min(SubDt, RemainingTime);
		}

		for (int32 i = 0; i < NUM_WHEELSETS; ++i)
		{
			int32 WheelsetIdx = i;

			auto DerivFunc = [this, WheelsetIdx](const FRK4WheelsetState& InState, float InTime) -> FRK4WheelsetDerivative
			{
				return this->ComputeWheelsetRK4Derivative(InState, InTime, WheelsetIdx);
			};

			RK4WheelsetStates[i] = TRK4Integrator<FRK4WheelsetState, FRK4WheelsetDerivative>::Integrate(
				RK4WheelsetStates[i], LocalTime, SubDt, DerivFunc
			);
		}

		auto CarBodyDerivFunc = [this](const FRK4CarBodyState& InState, float InTime) -> FRK4CarBodyDerivative
		{
			return this->ComputeCarBodyRK4Derivative(InState, InTime);
		};

		RK4CarBodyState = TRK4Integrator<FRK4CarBodyState, FRK4CarBodyDerivative>::Integrate(
			RK4CarBodyState, LocalTime, SubDt, CarBodyDerivFunc
		);

		if (bEnableVelocityClamping)
		{
			ClampVelocitiesToLimits();
		}

		if (bEnableEnergyMonitor)
		{
			ApplyEnergyCorrection();
		}

		RemainingTime -= SubDt;
		LocalTime += SubDt;
		TotalSubstepsThisFrame++;

		if (TotalSubstepsThisFrame >= RK4MaxSubstepsPerFrame)
		{
			UE_LOG(LogHSRTrackDynamics, Warning,
				TEXT("RK4 max substeps reached (%d), remaining time %.3e s"),
				RK4MaxSubstepsPerFrame, RemainingTime);
			break;
		}
	}

	TotalMechanicalEnergy = ComputeTotalMechanicalEnergy();

	if (NumericalStabilizer)
	{
		NumericalStabilizer->UpdateScalingFactors(DeltaTime, DominantFrequency);
		CurrentStiffnessScale = NumericalStabilizer->CurrentStiffnessScale;
		CurrentDampingScale = NumericalStabilizer->CurrentDampingScale;
	}

	if (TotalSubstepsThisFrame % 100 == 0 && TotalSubstepsThisFrame > 0)
	{
		UE_LOG(LogHSRTrackDynamics, Verbose,
			TEXT("RK4 step: %d substeps, speed=%.1f m/s, E=%.1f kJ, min dt=%.3e s"),
			TotalSubstepsThisFrame,
			RK4CarBodyState.Velocity.X,
			TotalMechanicalEnergy / 1000.0f,
			DeltaTime / TotalSubstepsThisFrame);
	}
}

FRK4WheelsetDerivative ATrainMBSVehicle::ComputeWheelsetRK4Derivative(
	const FRK4WheelsetState& State, float Time, int32 WheelsetIdx)
{
	FRK4WheelsetDerivative Deriv;

	Deriv.LinearVelocity = State.Velocity;
	Deriv.AngularVelocity = State.AngularVelocity;
	Deriv.SpinVelocity = State.WheelSpinVelocity;

	FVector Gravity = FVector(0.0f, 0.0f, -9.81f);

	FVector SuspensionForce, SuspensionTorque;
	ComputeSuspensionForcesRK4(WheelsetIdx, State, Time, SuspensionForce, SuspensionTorque);

	FVector ContactForce = FVector::ZeroVector;
	float EffectiveRadius = WheelRadius - 0.05f * FMath::Abs(State.Position.Y);
	float RailHeight = 0.0f;
	float Penetration = RailHeight + EffectiveRadius - State.Position.Z;

	if (Penetration > 0.0f)
	{
		float NormalForce;

		if (HighSpeedContactSolver)
		{
			NormalForce = HighSpeedContactSolver->ComputeHertzNormalForce(Penetration);
		}
		else
		{
			float HertzExponent = 1.5f;
			float HertzConst = (4.0f / 3.0f) * 1.15e11f * FMath::Sqrt(EffectiveRadius * 0.5f);
			NormalForce = HertzConst * FMath::Pow(Penetration, HertzExponent);
		}

		NormalForce *= CurrentStiffnessScale;

		ContactForce.Z = NormalForce;

		float Vx = State.Velocity.X;
		float Vy = State.Velocity.Y;
		float OmegaR = State.WheelSpinVelocity * EffectiveRadius;
		float V = FMath::Sqrt(Vx * Vx + Vy * Vy);

		if (V > 0.1f)
		{
			float CreepX = (Vx - OmegaR) / V;
			float CreepY = Vy / V;

			float Damping = PrimarySuspension.AxleBoxDampingZ * CurrentDampingScale * 0.01f;
			ContactForce.X = -Damping * V * CreepX * 10.0f;
			ContactForce.Y = -Damping * V * CreepY * 5.0f;

			float FrictionLimit = 0.35f * NormalForce;
			float TangMag = FMath::Sqrt(ContactForce.X * ContactForce.X + ContactForce.Y * ContactForce.Y);
			if (TangMag > FrictionLimit && TangMag > KINDA_SMALL_NUMBER)
			{
				float Scale = FrictionLimit / TangMag;
				ContactForce.X *= Scale;
				ContactForce.Y *= Scale;
			}
		}

		float DampingForce = -PrimarySuspension.AxleBoxDampingZ * State.Velocity.Z * CurrentDampingScale;
		ContactForce.Z += DampingForce;
	}

	FVector TotalForce = Gravity * WheelsetMass + SuspensionForce + ContactForce;
	Deriv.LinearAcceleration = TotalForce / WheelsetMass;

	float SpinTorque = -ContactForce.X * EffectiveRadius;
	Deriv.SpinAcceleration = SpinTorque / WheelsetInertiaYY;

	FVector Torque = FVector::ZeroVector;
	Torque.X += -SuspensionForce.Y * EffectiveRadius;
	Torque.Y += -ContactForce.X * (State.Position.Z - RailHeight);
	Torque.Z += SuspensionForce.Y * WheelRadius * 0.5f;

	Deriv.AngularAcceleration = FVector(
		Torque.X / WheelsetInertiaXX,
		Torque.Y / WheelsetInertiaYY,
		Torque.Z / WheelsetInertiaZZ
	);

	return Deriv;
}

FRK4CarBodyDerivative ATrainMBSVehicle::ComputeCarBodyRK4Derivative(
	const FRK4CarBodyState& State, float Time)
{
	FRK4CarBodyDerivative Deriv;

	Deriv.LinearVelocity = State.Velocity;
	Deriv.AngularVelocity = State.AngularVelocity;

	FVector Gravity = FVector(0.0f, 0.0f, -9.81f);
	FVector TotalForce = Gravity * CarBodyMass;
	FVector TotalTorque = FVector::ZeroVector;

	for (int32 BogieIdx = 0; BogieIdx < NUM_BOGIES; ++BogieIdx)
	{
		float BogieOffset = (BogieIdx == 0) ? -BogieHalfSpacing : BogieHalfSpacing;

		float RelVertical = 0.0f;
		float RelVerticalVel = 0.0f;
		for (int32 ws = 0; ws < NUM_WHEELSETS_PER_BOGIE; ++ws)
		{
			int32 WSIdx = BogieIdx * NUM_WHEELSETS_PER_BOGIE + ws;
			RelVertical += (RK4WheelsetStates[WSIdx].Position.Z - State.Position.Z - SecondaryVerticalOffset);
			RelVerticalVel += (RK4WheelsetStates[WSIdx].Velocity.Z - State.Velocity.Z);
		}
		RelVertical /= NUM_WHEELSETS_PER_BOGIE;
		RelVerticalVel /= NUM_WHEELSETS_PER_BOGIE;

		float AirSpringForce = SecondarySuspension.AirSpringStiffnessZ * RelVertical
			+ SecondarySuspension.AirSpringDampingZ * RelVerticalVel;
		AirSpringForce *= CurrentStiffnessScale;
		AirSpringForce *= CurrentDampingScale;

		FVector BogieForce = FVector(0.0f, 0.0f, -AirSpringForce);
		TotalForce += BogieForce;

		FVector ForcePos = State.Position + FVector(BogieOffset, 0.0f, SecondaryVerticalOffset);
		FVector Radius = ForcePos - State.Position;
		TotalTorque += FVector::CrossProduct(Radius, BogieForce);

		float RelLateral = 0.0f;
		for (int32 ws = 0; ws < NUM_WHEELSETS_PER_BOGIE; ++ws)
		{
			int32 WSIdx = BogieIdx * NUM_WHEELSETS_PER_BOGIE + ws;
			RelLateral += (RK4WheelsetStates[WSIdx].Position.Y - State.Position.Y);
		}
		RelLateral /= NUM_WHEELSETS_PER_BOGIE;

		float LatDampingForce = -SecondarySuspension.AirSpringDampingY * (RK4WheelsetStates[BogieIdx * 2].Velocity.Y - State.Velocity.Y) * CurrentDampingScale;
		TotalForce.Y += LatDampingForce;
	}

	Deriv.LinearAcceleration = TotalForce / CarBodyMass;

	FVector Inertia(CarBodyInertiaXX, CarBodyInertiaYY, CarBodyInertiaZZ);
	Deriv.AngularAcceleration = FVector(
		TotalTorque.X / Inertia.X,
		TotalTorque.Y / Inertia.Y,
		TotalTorque.Z / Inertia.Z
	);

	return Deriv;
}

void ATrainMBSVehicle::ComputeSuspensionForcesRK4(
	int32 WheelsetIdx,
	const FRK4WheelsetState& WSState,
	float Time,
	FVector& OutLinearForce,
	FVector& OutTorque)
{
	OutLinearForce = FVector::ZeroVector;
	OutTorque = FVector::ZeroVector;

	int32 BogieIdx = WheelsetIdx / NUM_WHEELSETS_PER_BOGIE;
	float BogieOffset = (BogieIdx == 0) ? -BogieHalfSpacing : BogieHalfSpacing;

	FVector BogiePos = RK4CarBodyState.Position + FVector(BogieOffset, 0.0f, -SecondaryVerticalOffset);
	FVector BogieVel = RK4CarBodyState.Velocity;

	float RelDispZ = WSState.Position.Z - BogiePos.Z - PrimaryVerticalOffset;
	float RelVelZ = WSState.Velocity.Z - BogieVel.Z;

	float PrimaryForceZ = -PrimarySuspension.AxleBoxSpringStiffnessZ * RelDispZ * CurrentStiffnessScale
		- PrimarySuspension.AxleBoxDampingZ * RelVelZ * CurrentDampingScale;
	OutLinearForce.Z += PrimaryForceZ;

	float RelDispY = WSState.Position.Y - BogiePos.Y;
	float RelVelY = WSState.Velocity.Y - BogieVel.Y;

	float PrimaryForceY = -PrimarySuspension.AxleBoxSpringStiffnessY * RelDispY * CurrentStiffnessScale
		- PrimarySuspension.AxleBoxDampingY * RelVelY * CurrentDampingScale;
	OutLinearForce.Y += PrimaryForceY;

	float BumperGap = SecondarySuspension.LateralBumperGap;
	if (FMath::Abs(RelDispY) > BumperGap)
	{
		float Penetration = FMath::Abs(RelDispY) - BumperGap;
		float Sign = FMath::Sign(RelDispY);
		OutLinearForce.Y += -Sign * SecondarySuspension.LateralBumperStiffness * Penetration * CurrentStiffnessScale;
	}

	float RelDispX = WSState.Position.X - BogiePos.X;
	float RelVelX = WSState.Velocity.X - BogieVel.X;
	float PrimaryForceX = -PrimarySuspension.AxleBoxSpringStiffnessX * RelDispX * CurrentStiffnessScale
		- PrimarySuspension.AxleBoxDampingX * RelVelX * CurrentDampingScale;
	OutLinearForce.X += PrimaryForceX;

	OutTorque.X += -PrimarySuspension.PrimaryVerticalStiffnessRoll * WSState.Rotation.Pitch * CurrentStiffnessScale
		- PrimarySuspension.PrimaryDampingRoll * WSState.AngularVelocity.X * CurrentDampingScale;

	OutTorque.Z += -PrimarySuspension.AxleBoxSpringStiffnessY * AxleHalfSpacing * AxleHalfSpacing * WSState.Rotation.Yaw * CurrentStiffnessScale;
}

float ATrainMBSVehicle::ComputeAdaptiveSubstepSize(
	float BaseDt,
	const FRK4WheelsetState& State) const
{
	float ImpactSpeed = FMath::Abs(State.Velocity.Z);

	if (ImpactSpeed > ImpactVelocityThreshold)
	{
		float Scale = ImpactVelocityThreshold / FMath::Max(ImpactSpeed, ImpactVelocityThreshold * 0.1f);
		return BaseDt * Scale * 0.05f;
	}

	float NaturalFreq = FMath::Sqrt(PrimarySuspension.AxleBoxSpringStiffnessZ * CurrentStiffnessScale / WheelsetMass);
	if (NaturalFreq > KINDA_SMALL_NUMBER)
	{
		float Period = 2.0f * PI / NaturalFreq;
		float DesiredDt = Period / 50.0f;
		return FMath::Min(BaseDt, DesiredDt);
	}

	return BaseDt;
}

float ATrainMBSVehicle::ComputeTotalMechanicalEnergy() const
{
	float TotalKE = 0.0f;

	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		TotalKE += ComputeWheelsetKineticEnergy(i);
	}

	TotalKE += ComputeCarBodyKineticEnergy();
	TotalKE += ComputePotentialEnergy();

	return TotalKE;
}

float ATrainMBSVehicle::ComputeWheelsetKineticEnergy(int32 WheelsetIdx) const
{
	if (!RK4WheelsetStates.IsValidIndex(WheelsetIdx)) return 0.0f;

	const FRK4WheelsetState& WS = RK4WheelsetStates[WheelsetIdx];
	float LinearKE = 0.5f * WheelsetMass * WS.Velocity.SizeSquared();
	float AngularKE = 0.5f * (
		WheelsetInertiaXX * WS.AngularVelocity.X * WS.AngularVelocity.X +
		WheelsetInertiaYY * WS.AngularVelocity.Y * WS.AngularVelocity.Y +
		WheelsetInertiaZZ * WS.AngularVelocity.Z * WS.AngularVelocity.Z
	);
	float SpinKE = 0.5f * WheelsetInertiaYY * WS.WheelSpinVelocity * WS.WheelSpinVelocity;

	return LinearKE + AngularKE + SpinKE;
}

float ATrainMBSVehicle::ComputeCarBodyKineticEnergy() const
{
	float LinearKE = 0.5f * CarBodyMass * RK4CarBodyState.Velocity.SizeSquared();
	float AngularKE = 0.5f * (
		CarBodyInertiaXX * RK4CarBodyState.AngularVelocity.X * RK4CarBodyState.AngularVelocity.X +
		CarBodyInertiaYY * RK4CarBodyState.AngularVelocity.Y * RK4CarBodyState.AngularVelocity.Y +
		CarBodyInertiaZZ * RK4CarBodyState.AngularVelocity.Z * RK4CarBodyState.AngularVelocity.Z
	);
	return LinearKE + AngularKE;
}

float ATrainMBSVehicle::ComputePotentialEnergy() const
{
	float PE = 0.0f;

	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		PE += WheelsetMass * 9.81f * RK4WheelsetStates[i].Position.Z;
	}

	PE += CarBodyMass * 9.81f * RK4CarBodyState.Position.Z;

	float DeflectionPE = 0.0f;
	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		int32 BogieIdx = i / NUM_WHEELSETS_PER_BOGIE;
		float BogieOffset = (BogieIdx == 0) ? -BogieHalfSpacing : BogieHalfSpacing;
		float BogieZ = RK4CarBodyState.Position.Z - SecondaryVerticalOffset;

		float DeltaZ = RK4WheelsetStates[i].Position.Z - BogieZ - PrimaryVerticalOffset;
		DeflectionPE += 0.5f * PrimarySuspension.AxleBoxSpringStiffnessZ * DeltaZ * DeltaZ;
	}
	PE += DeflectionPE;

	return PE;
}

void ATrainMBSVehicle::ClampVelocitiesToLimits()
{
	float SpeedLimitMs = SpeedLimitKmh / 3.6f;

	for (int32 i = 0; i < NUM_WHEELSETS; ++i)
	{
		FRK4WheelsetState& WS = RK4WheelsetStates[i];
		float Speed = WS.Velocity.Size();

		if (Speed > SpeedLimitMs * 2.0f)
		{
			WS.Velocity = WS.Velocity.GetSafeNormal() * SpeedLimitMs;
			VelocityClampCount++;
		}

		float AngularSpeed = WS.AngularVelocity.Size();
		if (AngularSpeed > MaxAngularVelocity)
		{
			WS.AngularVelocity = WS.AngularVelocity.GetSafeNormal() * MaxAngularVelocity;
			VelocityClampCount++;
		}

		float MaxSpin = SpeedLimitMs * 2.0f / WheelRadius;
		if (FMath::Abs(WS.WheelSpinVelocity) > MaxSpin)
		{
			WS.WheelSpinVelocity = FMath::Sign(WS.WheelSpinVelocity) * MaxSpin;
			VelocityClampCount++;
		}
	}

	float CarSpeed = RK4CarBodyState.Velocity.Size();
	if (CarSpeed > SpeedLimitMs * 2.0f)
	{
		RK4CarBodyState.Velocity = RK4CarBodyState.Velocity.GetSafeNormal() * SpeedLimitMs;
		VelocityClampCount++;
	}
}

void ATrainMBSVehicle::ApplyEnergyCorrection()
{
	if (!bEnableEnergyMonitor) return;

	static float LastEnergy = 0.0f;
	if (LastEnergy < KINDA_SMALL_NUMBER)
	{
		LastEnergy = ComputeTotalMechanicalEnergy();
		return;
	}

	float CurrentEnergy = ComputeTotalMechanicalEnergy();
	float EnergyRatio = 0.0f;

	if (LastEnergy > KINDA_SMALL_NUMBER)
	{
		EnergyRatio = (CurrentEnergy - LastEnergy) / LastEnergy;
	}

	if (EnergyRatio > MaxEnergyGainPerSubstep)
	{
		float MaxAllowedEnergy = LastEnergy * (1.0f + MaxEnergyGainPerSubstep);
		float Scale = FMath::Sqrt(MaxAllowedEnergy / FMath::Max(CurrentEnergy, KINDA_SMALL_NUMBER));

		for (int32 i = 0; i < NUM_WHEELSETS; ++i)
		{
			RK4WheelsetStates[i].Velocity *= Scale;
			RK4WheelsetStates[i].AngularVelocity *= Scale;
			RK4WheelsetStates[i].WheelSpinVelocity *= Scale;
		}
		RK4CarBodyState.Velocity *= Scale;
		RK4CarBodyState.AngularVelocity *= Scale;

		EnergyCorrectionCount++;

		if (EnergyRatio > MaxEnergyGainPerSubstep * 10.0f)
		{
			UE_LOG(LogHSRTrackDynamics, Warning,
				TEXT("Energy spike detected: ratio=%.6f, corrections=%d, emergency damping applied"),
				EnergyRatio, EnergyCorrectionCount);
		}
	}

	LastEnergy = CurrentEnergy;
}

bool ATrainMBSVehicle::CheckStabilityCondition()
{
	float NaturalFreq = FMath::Sqrt(PrimarySuspension.AxleBoxSpringStiffnessZ * CurrentStiffnessScale / WheelsetMass);
	float Period = 2.0f * PI / NaturalFreq;
	float AvgSubstep = RK4BaseSubstep;
	if (TotalSubstepsThisFrame > 0)
	{
		AvgSubstep = GetWorld()->GetDeltaSeconds() / TotalSubstepsThisFrame;
	}

	float StepsPerPeriod = Period / AvgSubstep;
	bool bStable = StepsPerPeriod >= 20.0f;

	if (!bStable)
	{
		UE_LOG(LogHSRTrackDynamics, Warning,
			TEXT("Stability warning: %.1f steps/period (need >= 20), dominant freq=%.1f Hz"),
			StepsPerPeriod, NaturalFreq / (2.0f * PI));
	}

	return bStable;
}
