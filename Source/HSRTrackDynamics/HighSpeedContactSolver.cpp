#include "HighSpeedContactSolver.h"
#include "HSRTrackDynamics.h"

UHighSpeedContactSolver::UHighSpeedContactSolver()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UHighSpeedContactSolver::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	AdvanceSimulation(DeltaTime);
}

void UHighSpeedContactSolver::InitializeSolver()
{
	WheelsetState.Position = InitialWheelPosition;
	WheelsetState.Velocity = InitialWheelVelocity;
	WheelsetState.WheelSpinVelocity = InitialWheelSpinVelocity;
	WheelsetState.Rotation = FQuat::Identity;
	WheelsetState.AngularVelocity = FVector::ZeroVector;

	if (FMath::Abs(InitialWheelSpinVelocity) < KINDA_SMALL_NUMBER &&
		FMath::Abs(InitialWheelVelocity.X) > KINDA_SMALL_NUMBER)
	{
		WheelsetState.WheelSpinVelocity = InitialWheelVelocity.X / WheelRadius;
	}

	TotalSimulatedTime = 0.0f;
	SubstepDiagnostics = FContactSubstepDiagnostics();

	UE_LOG(LogHSRTrackDynamics, Log,
		TEXT("HighSpeedContactSolver initialized: wheel radius=%.3f m, mass=%.1f kg, base substep=%.1e s"),
		WheelRadius, WheelsetMass, SolverParams.BaseSubstepTime);
}

void UHighSpeedContactSolver::ResetSolver()
{
	InitializeSolver();
}

void UHighSpeedContactSolver::AdvanceSimulation(float DeltaTime)
{
	if (DeltaTime < KINDA_SMALL_NUMBER) return;

	float RemainingTime = DeltaTime;
	float CurrentDt = SolverParams.BaseSubstepTime;
	SubstepDiagnostics.SubstepCount = 0;
	SubstepDiagnostics.MaxContactForce = 0.0f;
	SubstepDiagnostics.MaxPenetration = 0.0f;
	SubstepDiagnostics.MaxPressure = 0.0f;
	SubstepDiagnostics.ContactIterations = 0;
	SubstepDiagnostics.MinSubstepDt = CurrentDt;
	SubstepDiagnostics.EnergyDissipated = 0.0f;

	while (RemainingTime > KINDA_SMALL_NUMBER)
	{
		float SubDt = FMath::Min(CurrentDt, RemainingTime);
		float LocalTime = TotalSimulatedTime + (DeltaTime - RemainingTime);

		if (SolverParams.bUseAdaptiveSubstepping)
		{
			SubDt = FMath::Clamp(
				ComputeAdaptiveSubstep(SubDt, WheelsetState),
				SolverParams.MinSubstepTime,
				SolverParams.BaseSubstepTime
			);
			SubDt = FMath::Min(SubDt, RemainingTime);
		}

		SubstepDiagnostics.MinSubstepDt = FMath::Min(SubstepDiagnostics.MinSubstepDt, SubDt);

		FRK4WheelsetState OldState = WheelsetState;

		TRK4Integrator<FRK4WheelsetState, FRK4WheelsetDerivative>::FDerivativeFunc DerivFunc =
			[this](const FRK4WheelsetState& InState, float InTime) -> FRK4WheelsetDerivative
		{
			return this->ComputeWheelsetDerivative(InState, InTime);
		};

		WheelsetState = TRK4Integrator<FRK4WheelsetState, FRK4WheelsetDerivative>::Integrate(
			OldState, LocalTime, SubDt, DerivFunc
		);

		if (SolverParams.MaxContactIterations > 0)
		{
			for (int32 i = 0; i < SolverParams.MaxContactIterations; ++i)
			{
				FVector ContactForce = ComputeContactForce(WheelsetState, LocalTime + SubDt);
				float ForceMag = ContactForce.Size();

				PerformContactIteration(WheelsetState, LocalTime + SubDt, SubDt, i);
				SubstepDiagnostics.ContactIterations++;

				if (ForceMag < SolverParams.ContactForceTolerance &&
					FMath::Abs(WheelsetState.Position.Z - (GetRailHeightAt(WheelsetState.Position.X, WheelsetState.Position.Y) + WheelRadius))
					< SolverParams.PenetrationTolerance)
				{
					break;
				}
			}
		}

		ApplyNumericalDamping(WheelsetState, SubDt);

		FVector CurrentContactForce = ComputeContactForce(WheelsetState, LocalTime + SubDt);
		SubstepDiagnostics.MaxContactForce = FMath::Max(
			SubstepDiagnostics.MaxContactForce, CurrentContactForce.Size());
		SubstepDiagnostics.MaxPenetration = FMath::Max(
			SubstepDiagnostics.MaxPenetration,
			FMath::Max(0.0f, GetRailHeightAt(WheelsetState.Position.X, WheelsetState.Position.Y) + WheelRadius - WheelsetState.Position.Z)
		);

		float RelVelZ = WheelsetState.Velocity.Z;
		float EnergyDamp = RailDampingVertical * RelVelZ * RelVelZ * SubDt;
		SubstepDiagnostics.EnergyDissipated += FMath::Abs(EnergyDamp);

		RemainingTime -= SubDt;
		TotalSimulatedTime += SubDt;
		SubstepDiagnostics.SubstepCount++;

		if (SubstepDiagnostics.SubstepCount >= SolverParams.MaxSubstepsPerFrame)
		{
			UE_LOG(LogHSRTrackDynamics, Warning,
				TEXT("Max substeps reached (%d), remaining time %.3e s"),
				SolverParams.MaxSubstepsPerFrame, RemainingTime);
			break;
		}
	}

	ContactResult.NormalForce = ComputeContactForce(WheelsetState, TotalSimulatedTime).Size();
	ContactResult.MaxContactPressure = SubstepDiagnostics.MaxPressure;
	ContactResult.PenetrationDepth = SubstepDiagnostics.MaxPenetration;
	ContactResult.bInContact = SubstepDiagnostics.MaxPenetration > -KINDA_SMALL_NUMBER;
}

FRK4WheelsetDerivative UHighSpeedContactSolver::ComputeWheelsetDerivative(
	const FRK4WheelsetState& State, float Time)
{
	FRK4WheelsetDerivative Deriv;

	Deriv.LinearVelocity = State.Velocity;
	Deriv.AngularVelocity = State.AngularVelocity;
	Deriv.SpinVelocity = State.WheelSpinVelocity;

	FVector Gravity = FVector(0.0f, 0.0f, -9.81f);
	FVector ContactForce = ComputeContactForce(State, Time);

	FVector TotalForce = Gravity * WheelsetMass + ContactForce;
	Deriv.LinearAcceleration = TotalForce / WheelsetMass;

	float EffectiveRadius = ComputeEffectiveContactRadius(State.Position.Y);
	float TangentialForceX = -ContactForce.X;
	float SpinTorque = TangentialForceX * EffectiveRadius;
	Deriv.SpinAcceleration = SpinTorque / WheelsetInertiaY;

	FVector Torque = FVector::ZeroVector;
	Torque.Y += -ContactForce.X * (State.Position.Z - GetRailHeightAt(State.Position.X, State.Position.Y));
	Torque.Z += ContactForce.Y * WheelRadius;

	Deriv.AngularAcceleration = FVector(
		Torque.X / WheelsetInertiaXZ,
		Torque.Y / WheelsetInertiaY,
		Torque.Z / WheelsetInertiaXZ
	);

	return Deriv;
}

FVector UHighSpeedContactSolver::ComputeContactForce(const FRK4WheelsetState& State, float Time)
{
	FVector ContactForce = FVector::ZeroVector;

	float RailHeight = GetRailHeightAt(State.Position.X, State.Position.Y);
	float WheelBottomZ = State.Position.Z - WheelRadius;
	float Penetration = RailHeight - WheelBottomZ;

	if (Penetration <= -KINDA_SMALL_NUMBER)
	{
		return FVector::ZeroVector;
	}

	float NormalForce = ComputeHertzNormalForce(FMath::Max(0.0f, Penetration));

	FVector RailNormal = GetRailNormalAt(State.Position.X, State.Position.Y);
	FVector NormalDirection = FVector(0.0f, 0.0f, 1.0f);
	ContactForce = NormalDirection * NormalForce;

	FVector TangentialCreepForces = ComputeTangentialCreepForces(NormalForce, State);
	ContactForce += TangentialCreepForces;

	if (SolverParams.bUsePenaltyMethod && Penetration > 0.0f)
	{
		float PenaltyForce = Penetration * RailStiffnessVertical * SolverParams.PenaltyStiffnessMultiplier;
		ContactForce.Z += PenaltyForce;
	}

	if (SubstepDiagnostics.MaxPressure < 1.0f)
	{
		float A, B;
		float CombinedR = ComputeEffectiveContactRadius(State.Position.Y);
		float EStar = CombinedElasticModulus;
		A = FMath::Pow(3.0f * NormalForce * CombinedR / (4.0f * EStar), 1.0f / 3.0f);
		B = A * 0.6f;
		float Pressure = (3.0f * NormalForce) / (2.0f * PI * A * B);
		SubstepDiagnostics.MaxPressure = FMath::Max(SubstepDiagnostics.MaxPressure, Pressure);
	}

	return ContactForce;
}

float UHighSpeedContactSolver::ComputeHertzNormalForce(float Penetration)
{
	if (Penetration <= 0.0f) return 0.0f;

	float CombinedR = WheelRadius * 0.5f;
	float EStar = CombinedElasticModulus;

	if (!SolverParams.bNonlinearHertzContact)
	{
		return RailStiffnessVertical * Penetration;
	}

	float N = SolverParams.HertzExponent;
	float HertzConstant = (4.0f / 3.0f) * EStar * FMath::Sqrt(CombinedR);
	float Force = HertzConstant * FMath::Pow(Penetration, N);

	return Force;
}

float UHighSpeedContactSolver::ComputeHertzPenetration(float NormalForce)
{
	if (NormalForce <= 0.0f) return 0.0f;

	float CombinedR = WheelRadius * 0.5f;
	float EStar = CombinedElasticModulus;
	float N = SolverParams.HertzExponent;
	float HertzConstant = (4.0f / 3.0f) * EStar * FMath::Sqrt(CombinedR);

	return FMath::Pow(NormalForce / HertzConstant, 1.0f / N);
}

FVector UHighSpeedContactSolver::ComputeTangentialCreepForces(float NormalForce, const FRK4WheelsetState& State)
{
	if (NormalForce <= KINDA_SMALL_NUMBER) return FVector::ZeroVector;

	float EffectiveRadius = ComputeEffectiveContactRadius(State.Position.Y);
	float Vx = State.Velocity.X;
	float Vy = State.Velocity.Y;

	float OmegaR = State.WheelSpinVelocity * EffectiveRadius;
	float V = FMath::Sqrt(Vx * Vx + Vy * Vy);
	if (V < 0.1f) return FVector::ZeroVector;

	float CreepX = (Vx - OmegaR) / V;
	float CreepY = Vy / V;

	float CombinedR = EffectiveRadius;
	float EStar = CombinedElasticModulus;
	float SemiA = FMath::Pow(3.0f * NormalForce * CombinedR / (4.0f * EStar), 1.0f / 3.0f);
	float SemiB = SemiA * 0.6f;

	float C11 = 4.12f;
	float C22 = 3.67f;

	float AB = SemiA * SemiB;
	float StiffnessFactor = 2.0f * AB * EStar;

	float Fx = -StiffnessFactor * C11 * CreepX;
	float Fy = -StiffnessFactor * C22 * CreepY;

	float FrictionLimit = 0.35f * NormalForce;
	float TangForceMag = FMath::Sqrt(Fx * Fx + Fy * Fy);

	if (TangForceMag > FrictionLimit && TangForceMag > KINDA_SMALL_NUMBER)
	{
		float Scale = FrictionLimit / TangForceMag;
		Fx *= Scale;
		Fy *= Scale;
	}

	float DampingX = RailDampingVertical * 0.1f;
	float DampingY = RailDampingVertical * 0.05f;
	Fx += -DampingX * State.Velocity.X * 0.1f;
	Fy += -DampingY * State.Velocity.Y;

	return FVector(Fx, Fy, 0.0f);
}

float UHighSpeedContactSolver::GetRailHeightAt(float LongitudinalPos, float LateralPos) const
{
	float BaseHeight = RailSurfaceHeight;

	if (RailIrregularityAmplitude > 0.0f && RailIrregularityWavelength > 0.0f)
	{
		float WaveNum = 2.0f * PI / RailIrregularityWavelength;
		BaseHeight += RailIrregularityAmplitude * FMath::Sin(WaveNum * LongitudinalPos);
		BaseHeight += RailIrregularityAmplitude * 0.5f * FMath::Sin(2.0f * WaveNum * LongitudinalPos + 0.3f);
		BaseHeight += RailIrregularityAmplitude * 0.25f * FMath::Sin(4.0f * WaveNum * LongitudinalPos + 0.7f);
	}

	if (RailCurvatureRadius > KINDA_SMALL_NUMBER)
	{
		float Angle = LongitudinalPos / RailCurvatureRadius;
		BaseHeight += LateralPos * FMath::Tan(Angle * 0.1f);
	}

	return BaseHeight;
}

FVector UHighSpeedContactSolver::GetRailNormalAt(float LongitudinalPos, float LateralPos) const
{
	float Epsilon = 1.0e-4f;
	float H0 = GetRailHeightAt(LongitudinalPos, LateralPos);
	float Hx = GetRailHeightAt(LongitudinalPos + Epsilon, LateralPos);
	float Hy = GetRailHeightAt(LongitudinalPos, LateralPos + Epsilon);

	float dHdX = (Hx - H0) / Epsilon;
	float dHdY = (Hy - H0) / Epsilon;

	FVector Normal = FVector(-dHdX, -dHdY, 1.0f);
	Normal.Normalize();
	return Normal;
}

void UHighSpeedContactSolver::PerformContactIteration(
	FRK4WheelsetState& State, float Time, float Dt, int32 Iteration)
{
	float RailHeight = GetRailHeightAt(State.Position.X, State.Position.Y);
	float WheelBottomZ = State.Position.Z - WheelRadius;
	float Penetration = RailHeight - WheelBottomZ;

	if (Penetration <= -KINDA_SMALL_NUMBER) return;

	float Restitution = 0.1f;
	float VelZ = State.Velocity.Z;

	if (Penetration > 0.0f && VelZ < 0.0f)
	{
		float HertzForce = ComputeHertzNormalForce(Penetration);
		float Impulse = HertzForce * Dt;

		float DeltaVZ = Impulse / WheelsetMass;
		State.Velocity.Z += DeltaVZ;

		if (Iteration == 0)
		{
			float BounceVelocity = -VelZ * Restitution;
			if (State.Velocity.Z < BounceVelocity)
			{
				State.Velocity.Z = BounceVelocity;
			}
		}

		float Correction = Penetration * 0.5f;
		State.Position.Z += Correction;
	}
}

void UHighSpeedContactSolver::ApplyNumericalDamping(FRK4WheelsetState& State, float Dt)
{
	if (SolverParams.NumericalDampingCoeff <= KINDA_SMALL_NUMBER) return;

	float DampingFactor = 1.0f - SolverParams.NumericalDampingCoeff * Dt * 1000.0f;
	DampingFactor = FMath::Clamp(DampingFactor, 0.0f, 1.0f);

	State.Velocity *= DampingFactor;
	State.AngularVelocity *= DampingFactor;
	State.WheelSpinVelocity *= DampingFactor;

	float DampingZ = 1.0f - RailDampingVertical / (RailStiffnessVertical * Dt * 10.0f);
	DampingZ = FMath::Clamp(DampingZ, 0.9f, 1.0f);
	State.Velocity.Z *= DampingZ;
}

float UHighSpeedContactSolver::ComputeAdaptiveSubstep(float CurrentDt, const FRK4WheelsetState& State)
{
	float ImpactSpeed = FMath::Abs(State.Velocity.Z);

	if (ImpactSpeed > SolverParams.ImpactVelocityThreshold)
	{
		float Scale = SolverParams.ImpactVelocityThreshold / FMath::Max(ImpactSpeed, SolverParams.ImpactVelocityThreshold * 0.1f);
		return CurrentDt * Scale * 0.1f;
	}

	float AccelEst = RailStiffnessVertical * 0.001f / WheelsetMass;
	float NaturalFreq = FMath::Sqrt(RailStiffnessVertical / WheelsetMass);
	if (NaturalFreq > KINDA_SMALL_NUMBER)
	{
		float Period = 2.0f * PI / NaturalFreq;
		float DesiredDt = Period / 100.0f;
		return FMath::Min(CurrentDt, DesiredDt);
	}

	return CurrentDt;
}

float UHighSpeedContactSolver::ComputeEffectiveContactRadius(float LateralShift) const
{
	float ConeAngle = 0.05f;
	return WheelRadius - ConeAngle * FMath::Abs(LateralShift);
}
