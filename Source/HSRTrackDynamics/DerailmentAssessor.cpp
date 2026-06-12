#include "DerailmentAssessor.h"
#include "HSRTrackDynamics.h"

UDerailmentAssessor::UDerailmentAssessor()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
}

void UDerailmentAssessor::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	ComputeDerailmentAssessment();

	if (CurrentReport.OverallRiskLevel >= EDerailmentRiskLevel::Warning)
	{
		TimeSinceLastWarning = 0.0f;
		WarningCount++;
	}
	else
	{
		TimeSinceLastWarning += DeltaTime;
	}

	if (CurrentReport.bDerailmentImminent)
	{
		DangerCount++;
	}
}

void UDerailmentAssessor::ComputeDerailmentAssessment()
{
	int32 N = FMath::Min(WheelLateralForces.Num(), WheelVerticalForces.Num());
	N = FMath::Min(N, NumWheels);

	if (N <= 0)
	{
		CurrentReport = FDerailmentAssessmentReport();
		return;
	}

	CurrentReport.WheelData.SetNum(N);
	CurrentReport.MaxDerailmentCoefficient = 0.0f;
	CurrentReport.MaxLoadReductionRatio = 0.0f;
	CurrentReport.PeakVerticalAcceleration = 0.0f;
	CurrentReport.PeakLateralAcceleration = 0.0f;
	CurrentReport.VehicleSpeed = CurrentSpeed;
	CurrentReport.bDerailmentImminent = false;

	float NadalLimit = ComputeNadalLimit();

	EDerailmentRiskLevel WorstRisk = EDerailmentRiskLevel::Safe;

	for (int32 i = 0; i < N; ++i)
	{
		FWheelDerailmentData& Data = CurrentReport.WheelData[i];

		Data.LateralForceQ = WheelLateralForces.IsValidIndex(i) ? FMath::Abs(WheelLateralForces[i]) : 0.0f;
		Data.VerticalForceP = WheelVerticalForces.IsValidIndex(i) ? WheelVerticalForces[i] : StaticWheelLoad;
		Data.StaticWheelLoadP0 = StaticWheelLoad;
		Data.FlangeContactAngle = FlangeAngleDeg;
		Data.FrictionCoefficient = WheelRailFriction;
		Data.NadalLimit = NadalLimit;

		if (bEnableLowPassFilter)
		{
			float DeltaTime = 1.0f / 60.0f;
			float RawVertAccel = WheelVerticalAccelerations.IsValidIndex(i) ? WheelVerticalAccelerations[i] : 0.0f;
			float RawLatAccel = WheelLateralAccelerations.IsValidIndex(i) ? WheelLateralAccelerations[i] : 0.0f;

			if (!FilteredVertAccels.IsValidIndex(i))
			{
				FilteredVertAccels.Add(RawVertAccel);
				FilteredLatAccels.Add(RawLatAccel);
			}

			FilteredVertAccels[i] = ApplyLowPassFilter(RawVertAccel, FilteredVertAccels[i], AccelFilterCutoff, DeltaTime);
			FilteredLatAccels[i] = ApplyLowPassFilter(RawLatAccel, FilteredLatAccels[i], AccelFilterCutoff, DeltaTime);

			Data.VerticalAcceleration = FilteredVertAccels[i];
			Data.LateralAcceleration = FilteredLatAccels[i];
		}
		else
		{
			Data.VerticalAcceleration = WheelVerticalAccelerations.IsValidIndex(i) ? WheelVerticalAccelerations[i] : 0.0f;
			Data.LateralAcceleration = WheelLateralAccelerations.IsValidIndex(i) ? WheelLateralAccelerations[i] : 0.0f;
		}

		Data.DerailmentCoefficientQP = ComputeDerailmentCoefficient(Data.LateralForceQ, Data.VerticalForceP);
		Data.bExceedsNadal = Data.DerailmentCoefficientQP > NadalLimit;

		float ReducedLoad = StaticWheelLoad - Data.VerticalForceP;
		Data.WheelLoadReductionDeltaP = FMath::Max(0.0f, ReducedLoad);
		Data.LoadReductionRatio = ComputeLoadReductionRatio(Data.VerticalForceP, StaticWheelLoad);
		Data.bExceedsLoadReduction = Data.LoadReductionRatio > LoadReductionDanger;

		Data.QPRiskLevel = AssessQPRisk(Data.DerailmentCoefficientQP);
		Data.LoadReductionRiskLevel = AssessLoadReductionRisk(Data.LoadReductionRatio);
		Data.OverallRiskLevel = GetWorstRisk(Data.QPRiskLevel, Data.LoadReductionRiskLevel);

		EDerailmentRiskLevel AccelRisk = AssessAccelerationRisk(Data.VerticalAcceleration, Data.LateralAcceleration);
		Data.OverallRiskLevel = GetWorstRisk(Data.OverallRiskLevel, AccelRisk);

		CurrentReport.MaxDerailmentCoefficient = FMath::Max(CurrentReport.MaxDerailmentCoefficient, Data.DerailmentCoefficientQP);
		CurrentReport.MaxLoadReductionRatio = FMath::Max(CurrentReport.MaxLoadReductionRatio, Data.LoadReductionRatio);
		CurrentReport.PeakVerticalAcceleration = FMath::Max(CurrentReport.PeakVerticalAcceleration, FMath::Abs(Data.VerticalAcceleration));
		CurrentReport.PeakLateralAcceleration = FMath::Max(CurrentReport.PeakLateralAcceleration, FMath::Abs(Data.LateralAcceleration));

		WorstRisk = GetWorstRisk(WorstRisk, Data.OverallRiskLevel);

		if (Data.DerailmentCoefficientQP > NadalLimit || Data.LoadReductionRatio > LoadReductionDanger)
		{
			CurrentReport.bDerailmentImminent = true;
		}
	}

	CurrentReport.OverallRiskLevel = WorstRisk;

	{
		float DeltaTime = 1.0f / 60.0f;
		FilteredCarbodyVert = ApplyLowPassFilter(CarbodyVerticalAcceleration, FilteredCarbodyVert, AccelFilterCutoff, DeltaTime);
		FilteredCarbodyLat = ApplyLowPassFilter(CarbodyLateralAcceleration, FilteredCarbodyLat, AccelFilterCutoff, DeltaTime);
		CurrentReport.MaxCarbodyVerticalAccel = FMath::Abs(FilteredCarbodyVert);
		CurrentReport.MaxCarbodyLateralAccel = FMath::Abs(FilteredCarbodyLat);
	}

	PeakVerticalAccelHistory = FMath::Max(PeakVerticalAccelHistory, CurrentReport.PeakVerticalAcceleration);
	PeakLateralAccelHistory = FMath::Max(PeakLateralAccelHistory, CurrentReport.PeakLateralAcceleration);

	if (CurrentReport.bDerailmentImminent)
	{
		UE_LOG(LogHSRTrackDynamics, Error,
			TEXT("DERAILMENT IMMINENT! Q/P=%.3f (limit=%.3f), ΔP/P0=%.1f%%, VertAccel=%.2f g, Speed=%.1f km/h"),
			CurrentReport.MaxDerailmentCoefficient,
			NadalLimit,
			CurrentReport.MaxLoadReductionRatio * 100.0f,
			CurrentReport.PeakVerticalAcceleration / 9.81f,
			CurrentSpeed * 3.6f);
	}
	else if (CurrentReport.OverallRiskLevel >= EDerailmentRiskLevel::Warning)
	{
		UE_LOG(LogHSRTrackDynamics, Warning,
			TEXT("Derailment WARNING: Q/P=%.3f, ΔP/P0=%.1f%%, VertAccel=%.2f g, Speed=%.1f km/h"),
			CurrentReport.MaxDerailmentCoefficient,
			CurrentReport.MaxLoadReductionRatio * 100.0f,
			CurrentReport.PeakVerticalAcceleration / 9.81f,
			CurrentSpeed * 3.6f);
	}
}

float UDerailmentAssessor::ComputeNadalLimit() const
{
	float Alpha = FMath::DegreesToRadians(FlangeAngleDeg);
	float Mu = WheelRailFriction;
	float TanAlpha = FMath::Tan(Alpha);
	float Numerator = TanAlpha - Mu;
	float Denominator = 1.0f + Mu * TanAlpha;

	if (FMath::Abs(Denominator) < KINDA_SMALL_NUMBER) return 1.0f;

	return FMath::Max(Numerator / Denominator, 0.1f);
}

float UDerailmentAssessor::ComputeDerailmentCoefficient(float LateralForceQ, float VerticalForceP) const
{
	if (FMath::Abs(VerticalForceP) < 1.0f) return 100.0f;
	return FMath::Abs(LateralForceQ) / FMath::Abs(VerticalForceP);
}

float UDerailmentAssessor::ComputeLoadReductionRatio(float CurrentVerticalForce, float StaticLoad) const
{
	if (StaticLoad < KINDA_SMALL_NUMBER) return 0.0f;
	float Reduction = (StaticLoad - CurrentVerticalForce) / StaticLoad;
	return FMath::Clamp(Reduction, 0.0f, 2.0f);
}

EDerailmentRiskLevel UDerailmentAssessor::AssessQPRisk(float QP) const
{
	if (QP >= DangerQPRatio) return EDerailmentRiskLevel::Danger;
	if (QP >= WarningQPRatio) return EDerailmentRiskLevel::Warning;
	if (QP >= CautionQPRatio) return EDerailmentRiskLevel::Caution;
	return EDerailmentRiskLevel::Safe;
}

EDerailmentRiskLevel UDerailmentAssessor::AssessLoadReductionRisk(float Ratio) const
{
	if (Ratio >= LoadReductionDanger) return EDerailmentRiskLevel::Danger;
	if (Ratio >= LoadReductionWarning) return EDerailmentRiskLevel::Warning;
	if (Ratio >= LoadReductionCaution) return EDerailmentRiskLevel::Caution;
	return EDerailmentRiskLevel::Safe;
}

EDerailmentRiskLevel UDerailmentAssessor::AssessAccelerationRisk(float VertAccel, float LatAccel) const
{
	float VertG = FMath::Abs(VertAccel) / 9.81f;
	float LatG = FMath::Abs(LatAccel) / 9.81f;

	EDerailmentRiskLevel VertRisk = EDerailmentRiskLevel::Safe;
	if (VertG >= VerticalAccelDanger) VertRisk = EDerailmentRiskLevel::Danger;
	else if (VertG >= VerticalAccelWarning) VertRisk = EDerailmentRiskLevel::Warning;
	else if (VertG >= VerticalAccelCaution) VertRisk = EDerailmentRiskLevel::Caution;

	EDerailmentRiskLevel LatRisk = EDerailmentRiskLevel::Safe;
	if (LatG >= LateralAccelDanger) LatRisk = EDerailmentRiskLevel::Danger;
	else if (LatG >= LateralAccelWarning) LatRisk = EDerailmentRiskLevel::Warning;
	else if (LatG >= LateralAccelCaution) LatRisk = EDerailmentRiskLevel::Caution;

	return GetWorstRisk(VertRisk, LatRisk);
}

float UDerailmentAssessor::ApplyLowPassFilter(float RawValue, float PreviousFiltered, float CutoffHz, float DeltaTime)
{
	if (CutoffHz < KINDA_SMALL_NUMBER || DeltaTime < KINDA_SMALL_NUMBER) return RawValue;

	float RC = 1.0f / (2.0f * PI * CutoffHz);
	float Alpha = DeltaTime / (RC + DeltaTime);
	return PreviousFiltered + Alpha * (RawValue - PreviousFiltered);
}

FString UDerailmentAssessor::GetRiskLevelString(EDerailmentRiskLevel Level) const
{
	switch (Level)
	{
	case EDerailmentRiskLevel::Safe: return TEXT("SAFE");
	case EDerailmentRiskLevel::Caution: return TEXT("CAUTION");
	case EDerailmentRiskLevel::Warning: return TEXT("WARNING");
	case EDerailmentRiskLevel::Danger: return TEXT("DANGER");
	case EDerailmentRiskLevel::Critical: return TEXT("CRITICAL DERAILMENT");
	default: return TEXT("UNKNOWN");
	}
}

FString UDerailmentAssessor::GetSummaryString() const
{
	return FString::Printf(
		TEXT("Q/P=%.3f/%.3f | ΔP/P₀=%.1f%%/%.0f%% | VertAccel=%.2fg | LatAccel=%.2fg | Risk=%s | CarbodyVert=%.2fg"),
		CurrentReport.MaxDerailmentCoefficient,
		ComputeNadalLimit(),
		CurrentReport.MaxLoadReductionRatio * 100.0f,
		LoadReductionDanger * 100.0f,
		CurrentReport.PeakVerticalAcceleration / 9.81f,
		CurrentReport.PeakLateralAcceleration / 9.81f,
		*GetRiskLevelString(CurrentReport.OverallRiskLevel),
		CurrentReport.MaxCarbodyVerticalAccel / 9.81f
	);
}

void UDerailmentAssessor::ResetAssessor()
{
	CurrentReport = FDerailmentAssessmentReport();
	TimeSinceLastWarning = 0.0f;
	WarningCount = 0;
	DangerCount = 0;
	PeakVerticalAccelHistory = 0.0f;
	PeakLateralAccelHistory = 0.0f;
	FilteredVertAccels.Empty();
	FilteredLatAccels.Empty();
	FilteredCarbodyVert = 0.0f;
	FilteredCarbodyLat = 0.0f;
}

EDerailmentRiskLevel UDerailmentAssessor::GetWorstRisk(EDerailmentRiskLevel A, EDerailmentRiskLevel B) const
{
	return (static_cast<uint8>(A) >= static_cast<uint8>(B)) ? A : B;
}
