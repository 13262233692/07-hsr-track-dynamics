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
	return CarBodyState.LongitudinalVel;
}
