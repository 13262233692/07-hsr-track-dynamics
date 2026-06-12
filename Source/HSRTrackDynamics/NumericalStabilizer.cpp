#include "NumericalStabilizer.h"
#include "HSRTrackDynamics.h"

UNumericalStabilizer::UNumericalStabilizer()
{
	PrimaryComponentTick.bCanEverTick = false;
}

FStabilityAnalysisResult UNumericalStabilizer::AnalyzeStability(
	float DominantFrequency,
	float DominantStiffness,
	float DominantMass,
	float CurrentTimeStep)
{
	FStabilityAnalysisResult Result;
	Result.CurrentDt = CurrentTimeStep;
	Result.HighestFrequency = DominantFrequency;

	float Omega = 2.0f * PI * DominantFrequency;
	float NaturalOmega = FMath::Sqrt(DominantStiffness / FMath::Max(DominantMass, KINDA_SMALL_NUMBER));
	Result.HighestFrequency = FMath::Max(DominantFrequency, NaturalOmega / (2.0f * PI));

	float MaxOmega = FMath::Max(Omega, NaturalOmega);
	float MaxDt = (2.0f * PI / MaxOmega) / StiffnessConfig.MinStepsPerPeriod;
	Result.MaxAllowedDt = MaxDt * StiffnessConfig.SafetyFactor;

	if (MaxDt > KINDA_SMALL_NUMBER)
	{
		Result.StabilityRatio = CurrentTimeStep / MaxDt;
	}
	else
	{
		Result.StabilityRatio = 100.0f;
	}

	float Period = 2.0f * PI / MaxOmega;
	Result.StepsPerPeriod = Period / CurrentTimeStep;

	float DampingRatio = ComputeRayleighDamping(MaxOmega, DominantMass, DominantStiffness) /
		(2.0f * DominantMass * MaxOmega);
	Result.NumericalDampingRatio = DampingRatio;

	Result.bViolatesCourant = Result.StabilityRatio > 1.0f;

	if (Result.StabilityRatio < 0.5f)
	{
		Result.Condition = EStabilityCondition::Stable;
	}
	else if (Result.StabilityRatio < 0.8f)
	{
		Result.Condition = EStabilityCondition::Warning;
	}
	else if (Result.StabilityRatio < 1.0f)
	{
		Result.Condition = EStabilityCondition::Critical;
	}
	else
	{
		Result.Condition = EStabilityCondition::Unstable;
	}

	StabilityStatus = Result;
	return Result;
}

float UNumericalStabilizer::ComputeOptimalTimeStep(
	float DominantFrequency,
	float DominantStiffness,
	float DominantMass)
{
	float Omega = 2.0f * PI * DominantFrequency;
	float NaturalOmega = FMath::Sqrt(DominantStiffness / FMath::Max(DominantMass, KINDA_SMALL_NUMBER));
	float MaxOmega = FMath::Max(Omega, NaturalOmega);

	if (MaxOmega < KINDA_SMALL_NUMBER)
	{
		return 1.0f / 60.0f;
	}

	float MinPeriod = 2.0f * PI / MaxOmega;
	float OptimalDt = MinPeriod / StiffnessConfig.MinStepsPerPeriod * StiffnessConfig.SafetyFactor;

	return OptimalDt;
}

float UNumericalStabilizer::ComputeRayleighDamping(
	float AngularFrequency,
	float Mass,
	float Stiffness)
{
	float Alpha = StiffnessConfig.RayleighAlphaMass * Mass;
	float Beta = StiffnessConfig.RayleighBetaStiffness * Stiffness;
	return Alpha + Beta * AngularFrequency * AngularFrequency;
}

float UNumericalStabilizer::ComputeNumericalDampingRatio(
	float TimeStep,
	float AngularFrequency,
	float DampingRatio)
{
	float OmegaDt = AngularFrequency * TimeStep;
	float OmegaDtSq = OmegaDt * OmegaDt;

	if (OmegaDtSq < KINDA_SMALL_NUMBER) return 0.0f;

	float CoshOmega = FMath::Cos(OmegaDt);
	float SinhOmega = FMath::Sin(OmegaDt);

	float DecayFactor = FMath::Exp(-DampingRatio * OmegaDt);
	float NumericalDamping = -FMath::Log(DecayFactor) / OmegaDt;

	return NumericalDamping;
}

float UNumericalStabilizer::ComputeNonlinearDamperForce(
	float Velocity,
	float BaseDamping,
	float ScaleFactor)
{
	float AbsVel = FMath::Abs(Velocity);
	float SignVel = FMath::Sign(Velocity);

	if (!DamperConfig.bNonlinearDamper)
	{
		return -BaseDamping * ScaleFactor * Velocity;
	}

	float VMax = DamperConfig.VelocityForMaxDamping;
	float N = DamperConfig.DamperExponent;

	float VelRatio = FMath::Min(AbsVel / VMax, 1.0f);
	float ForceMag = BaseDamping * VMax * FMath::Pow(VelRatio, N) * ScaleFactor;

	return -SignVel * ForceMag;
}

FVector UNumericalStabilizer::ComputeNonlinearDamperForce3D(
	const FVector& Velocity,
	const FVector& BaseDamping,
	float ScaleFactor)
{
	return FVector(
		ComputeNonlinearDamperForce(Velocity.X, BaseDamping.X, ScaleFactor),
		ComputeNonlinearDamperForce(Velocity.Y, BaseDamping.Y, ScaleFactor),
		ComputeNonlinearDamperForce(Velocity.Z, BaseDamping.Z, ScaleFactor)
	);
}

void UNumericalStabilizer::UpdateEnergyBalance(
	float KineticEnergy,
	float PotentialEnergy,
	float DeltaTime)
{
	PreviousEnergy = TotalMechanicalEnergy;
	TotalMechanicalEnergy = KineticEnergy + PotentialEnergy;

	EnergyHistory[EnergyHistoryIndex] = TotalMechanicalEnergy;
	EnergyHistoryIndex = (EnergyHistoryIndex + 1) % 8;

	if (PreviousEnergy > KINDA_SMALL_NUMBER)
	{
		EnergyTrend = (TotalMechanicalEnergy - PreviousEnergy) / PreviousEnergy;
	}
	else
	{
		EnergyTrend = 0.0f;
	}

	if (StiffnessConfig.bEnableEnergyPreservation &&
		EnergyTrend > StiffnessConfig.MaxEnergyGainPerStep)
	{
		EmergencyCorrections++;
	}
}

float UNumericalStabilizer::ApplyEnergyCorrection(
	float Velocity,
	float Mass,
	float MaxEnergyGain)
{
	if (!StiffnessConfig.bEnableEnergyPreservation) return Velocity;

	float KE = 0.5f * Mass * Velocity * Velocity;
	float MaxKE = KE * (1.0f + MaxEnergyGain);

	if (KE > MaxKE && KE > KINDA_SMALL_NUMBER)
	{
		float Scale = FMath::Sqrt(MaxKE / KE);
		return Velocity * Scale;
	}

	return Velocity;
}

FVector UNumericalStabilizer::ApplyEnergyCorrection3D(
	const FVector& Velocity,
	float Mass,
	float MaxEnergyGain)
{
	if (!StiffnessConfig.bEnableEnergyPreservation) return Velocity;

	float KE = 0.5f * Mass * Velocity.SizeSquared();
	float MaxKE = KE * (1.0f + MaxEnergyGain);

	if (KE > MaxKE && KE > KINDA_SMALL_NUMBER)
	{
		float Scale = FMath::Sqrt(MaxKE / KE);
		return Velocity * Scale;
	}

	return Velocity;
}

void UNumericalStabilizer::UpdateScalingFactors(float DeltaTime, float DominantFrequency)
{
	float Omega = 2.0f * PI * DominantFrequency;

	if (StiffnessConfig.bAutoScaleStiffness)
	{
		float MaxOmega = FMath::Max(Omega, KINDA_SMALL_NUMBER);
		float Period = 2.0f * PI / MaxOmega;
		float StepsNeeded = StiffnessConfig.MinStepsPerPeriod / StiffnessConfig.SafetyFactor;
		float RequiredDt = Period / StepsNeeded;

		if (DeltaTime > RequiredDt && DeltaTime > KINDA_SMALL_NUMBER)
		{
			float Ratio = RequiredDt / DeltaTime;
			CurrentStiffnessScale = FMath::Clamp(
				Ratio * Ratio,
				StiffnessConfig.StiffnessScaleFloor,
				StiffnessConfig.StiffnessScaleCeiling
			);
		}
		else
		{
			CurrentStiffnessScale = 1.0f;
		}
	}
	else
	{
		CurrentStiffnessScale = 1.0f;
	}

	if (DamperConfig.bAdaptiveDamper)
	{
		float FreqRatio = DominantFrequency / DamperConfig.HighFrequencyThreshold;
		float Scale = 1.0f + (DamperConfig.DampingScaleAtHighFreq - 1.0f) *
			FMath::Clamp(FreqRatio, 0.0f, 1.0f);
		CurrentDampingScale = Scale;
	}
	else
	{
		CurrentDampingScale = 1.0f;
	}

	if (StabilityStatus.Condition == EStabilityCondition::Unstable ||
		StabilityStatus.Condition == EStabilityCondition::Critical)
	{
		StabilityViolationCount++;
		CurrentDampingScale *= 1.5f;
	}
}

void UNumericalStabilizer::ResetStabilizer()
{
	StabilityStatus = FStabilityAnalysisResult();
	CurrentStiffnessScale = 1.0f;
	CurrentDampingScale = 1.0f;
	TotalMechanicalEnergy = 0.0f;
	PreviousEnergy = 0.0f;
	EnergyTrend = 0.0f;
	StabilityViolationCount = 0;
	EmergencyCorrections = 0;
	EnergyHistoryIndex = 0;
	FMemory::Memzero(EnergyHistory, sizeof(EnergyHistory));
}

bool UNumericalStabilizer::CheckCourantFriedrichsLewy(
	float WaveSpeed,
	float ElementSize,
	float TimeStep) const
{
	if (ElementSize < KINDA_SMALL_NUMBER || TimeStep < KINDA_SMALL_NUMBER) return true;

	float CFLNumber = WaveSpeed * TimeStep / ElementSize;
	return CFLNumber <= 1.0f;
}

bool UNumericalStabilizer::CheckNewmarkStability(
	float Gamma,
	float Beta,
	float DampingRatio) const
{
	bool UnconditionallyStable = (Gamma >= 0.5f && Beta >= 0.25f * (Gamma + 0.5f) * (Gamma + 0.5f));
	return UnconditionallyStable;
}

float UNumericalStabilizer::ComputeEffectiveFrequency(float AngularFreq, float DampingRatio) const
{
	float OmegaD = AngularFreq * FMath::Sqrt(1.0f - DampingRatio * DampingRatio);
	return OmegaD / (2.0f * PI);
}

float UNumericalStabilizer::SmoothStep(float Edge0, float Edge1, float X) const
{
	float T = FMath::Clamp((X - Edge0) / (Edge1 - Edge0), 0.0f, 1.0f);
	return T * T * (3.0f - 2.0f * T);
}
