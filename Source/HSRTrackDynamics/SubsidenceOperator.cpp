#include "SubsidenceOperator.h"
#include "TrackGenerator.h"
#include "FastenerSpringGrid.h"
#include "HSRTrackDynamics.h"
#include "ProceduralMeshComponent.h"

ASubsidenceOperator::ASubsidenceOperator()
{
	PrimaryActorTick.bCanEverTick = false;
}

void ASubsidenceOperator::BeginPlay()
{
	Super::BeginPlay();
}

float FSubsidenceZone::ComputeSettlement(float DistanceAlongTrack) const
{
	float Span = EndDistance - StartDistance;
	if (Span < KINDA_SMALL_NUMBER) return 0.0f;

	float Norm = (DistanceAlongTrack - StartDistance) / Span;
	if (Norm < 0.0f || Norm > 1.0f) return 0.0f;

	if (!bAsymmetric)
	{
		float CurveVal = 0.0f;
		switch (CurveType)
		{
		case ESubsidenceCurveType::HalfSine:
			CurveVal = FMath::Sin(Norm * PI);
			break;
		case ESubsidenceCurveType::CubicPolynomial:
			CurveVal = 3.0f * Norm * Norm - 2.0f * Norm * Norm * Norm;
			break;
		case ESubsidenceCurveType::QuinticPolynomial:
			CurveVal = 10.0f * FMath::Pow(Norm, 3) - 15.0f * FMath::Pow(Norm, 4) + 6.0f * FMath::Pow(Norm, 5);
			break;
		case ESubsidenceCurveType::Gaussian:
			{
				float Mu = 0.5f;
				float Sigma = 0.2f;
				CurveVal = FMath::Exp(-0.5f * FMath::Pow((Norm - Mu) / Sigma, 2));
				float PeakVal = FMath::Exp(-0.5f * FMath::Pow((Mu - Mu) / Sigma, 2));
				CurveVal /= PeakVal;
			}
			break;
		case ESubsidenceCurveType::CosineTaper:
			CurveVal = 0.5f * (1.0f - FMath::Cos(2.0f * PI * Norm));
			break;
		default:
			CurveVal = FMath::Sin(Norm * PI);
			break;
		}

		return MaxDepth * CurveVal;
	}
	else
	{
		float LeftCurve = FMath::Sin(Norm * PI);
		float RightCurve = FMath::Sin(Norm * PI);

		if (bAsymmetric)
		{
			float ShiftedNorm = FMath::Clamp(Norm + AsymmetryOffset, 0.0f, 1.0f);
			RightCurve = FMath::Sin(ShiftedNorm * PI);
		}

		return MaxDepth * (LeftCurve * LeftDepthMultiplier + RightCurve * RightDepthMultiplier) * 0.5f;
	}
}

void ASubsidenceOperator::ApplySubsidenceZone(int32 ZoneIndex)
{
	if (!SubsidenceZones.IsValidIndex(ZoneIndex)) return;
	if (!TrackGeneratorRef || !FastenerGridRef) return;

	FSubsidenceZone& Zone = SubsidenceZones[ZoneIndex];
	Zone.bActive = true;
	Zone.AffectedLength = Zone.EndDistance - Zone.StartDistance;

	int32 TotalFasteners = TrackGeneratorRef->GetFastenerCount();
	float Spacing = TrackGeneratorRef->GetTotalFastenerSpacing();
	int32 Affected = 0;

	for (int32 i = 0; i < TotalFasteners; ++i)
	{
		float FastenerDist = i * Spacing * 0.5f;
		float Settlement = Zone.ComputeSettlement(FastenerDist);

		if (FMath::Abs(Settlement) > KINDA_SMALL_NUMBER)
		{
			Affected++;
		}
	}

	Zone.AffectedFastenerCount = Affected;

	UpdateStiffnessReductionMatrix();
	PropagateStiffnessToFastenerGrid();
	UpdateTrackMeshVertices();

	UE_LOG(LogHSRTrackDynamics, Log,
		TEXT("Subsidence zone applied: %.1f m ~ %.1f m, depth=%.1f mm, affected=%d fasteners"),
		Zone.StartDistance, Zone.EndDistance, Zone.MaxDepth * 1000.0f, Affected);
}

void ASubsidenceOperator::ApplyAllSubsidenceZones()
{
	for (int32 i = 0; i < SubsidenceZones.Num(); ++i)
	{
		ApplySubsidenceZone(i);
	}
}

void ASubsidenceOperator::AddSubsidenceZone(float StartDist, float EndDist, float MaxDepthMm, ESubsidenceCurveType CurveType)
{
	FSubsidenceZone NewZone;
	NewZone.StartDistance = StartDist;
	NewZone.EndDistance = EndDist;
	NewZone.MaxDepth = -MaxDepthMm / 1000.0f;
	NewZone.CurveType = CurveType;
	NewZone.bActive = false;
	SubsidenceZones.Add(NewZone);

	UE_LOG(LogHSRTrackDynamics, Log,
		TEXT("Subsidence zone added: %.1f m ~ %.1f m, depth=%.1f mm"),
		StartDist, EndDist, MaxDepthMm);
}

void ASubsidenceOperator::RemoveSubsidenceZone(int32 ZoneIndex)
{
	if (SubsidenceZones.IsValidIndex(ZoneIndex))
	{
		SubsidenceZones.RemoveAt(ZoneIndex);
		UpdateStiffnessReductionMatrix();
		PropagateStiffnessToFastenerGrid();
		UpdateTrackMeshVertices();
	}
}

void ASubsidenceOperator::ClearAllSubsidenceZones()
{
	SubsidenceZones.Empty();
	ResetAllSettlements();
}

void ASubsidenceOperator::UpdateStiffnessReductionMatrix()
{
	if (!TrackGeneratorRef || !FastenerGridRef) return;

	int32 TotalFasteners = FastenerGridRef->GetFastenerCount();
	float Spacing = TrackGeneratorRef->GetTotalFastenerSpacing();

	StiffnessReductionMatrix.SetNum(TotalFasteners);

	MaxSettlementDepth = 0.0f;
	MaxSettlementSlope = 0.0f;
	MaxStiffnessReductionApplied = 0.0f;
	MinPretensionRemaining = 1.0f;
	TotalAffectedFasteners = 0;

	FFastenerStiffnessParams OriginalParams = FastenerGridRef->StiffnessParams;

	for (int32 i = 0; i < TotalFasteners; ++i)
	{
		float FastenerDist = i * Spacing * 0.5f;

		float TotalSettlement = 0.0f;
		for (const auto& Zone : SubsidenceZones)
		{
			if (Zone.bActive)
			{
				TotalSettlement += Zone.ComputeSettlement(FastenerDist);
			}
		}

		float SettlementSlope = GetSettlementSlopeAtDistance(FastenerDist);

		FStiffnessReductionEntry& Entry = StiffnessReductionMatrix[i];
		Entry.FastenerIndex = i;
		Entry.SettlementAtFastener = TotalSettlement;
		Entry.SettlementSlope = SettlementSlope;

		float AbsSettlement = FMath::Abs(TotalSettlement);
		float SettlementMm = AbsSettlement * 1000.0f;

		float DepthReduction = FMath::Min(SettlementMm * StiffnessReductionPerMm, MaxStiffnessReduction);
		float SlopeReduction = FMath::Min(FMath::Abs(SettlementSlope) * SlopeReductionRate, 0.3f);

		float TotalReduction = FMath::Min(DepthReduction + SlopeReduction, MaxStiffnessReduction);

		if (bProgressiveDegradation)
		{
			TotalReduction = ComputeTimeDependentStiffness(TotalReduction, CurrentServiceYears);
		}

		Entry.StiffnessReductionFactor = 1.0f - TotalReduction;

		Entry.OriginalVerticalStiffness = OriginalParams.VerticalStiffness;
		Entry.ReducedVerticalStiffness = OriginalParams.VerticalStiffness * Entry.StiffnessReductionFactor;

		Entry.OriginalLateralStiffness = OriginalParams.LateralStiffness;
		Entry.ReducedLateralStiffness = OriginalParams.LateralStiffness * Entry.StiffnessReductionFactor;

		Entry.OriginalPretension = BasePretension;
		float PretensionLoss = SettlementMm * PretensionLossPerMm;
		if (bProgressiveDegradation)
		{
			PretensionLoss *= (1.0f + TimeHardeningFactor * CurrentServiceYears / DegradationYears);
		}
		float MinPretension = BasePretension * MinPretensionRatio;
		Entry.ReducedPretension = FMath::Max(BasePretension - PretensionLoss, MinPretension);

		if (AbsSettlement > KINDA_SMALL_NUMBER)
		{
			TotalAffectedFasteners++;

			MaxSettlementDepth = FMath::Max(MaxSettlementDepth, AbsSettlement);
			MaxSettlementSlope = FMath::Max(MaxSettlementSlope, FMath::Abs(SettlementSlope));
			MaxStiffnessReductionApplied = FMath::Max(MaxStiffnessReductionApplied, TotalReduction);
			MinPretensionRemaining = FMath::Min(MinPretensionRemaining, Entry.ReducedPretension / BasePretension);
		}
	}

	UE_LOG(LogHSRTrackDynamics, Log,
		TEXT("Stiffness matrix updated: %d affected, max depth=%.2f mm, max reduction=%.1f%%, min pretension=%.0f%%"),
		TotalAffectedFasteners,
		MaxSettlementDepth * 1000.0f,
		MaxStiffnessReductionApplied * 100.0f,
		MinPretensionRemaining * 100.0f);
}

void ASubsidenceOperator::PropagateStiffnessToFastenerGrid()
{
	if (!FastenerGridRef) return;

	int32 Updated = 0;
	for (const auto& Entry : StiffnessReductionMatrix)
	{
		if (Entry.FastenerIndex < 0) continue;
		if (FMath::Abs(Entry.SettlementAtFastener) < KINDA_SMALL_NUMBER) continue;

		if (FastenerGridRef->PhysicsConstraints.IsValidIndex(Entry.FastenerIndex))
		{
			UPhysicsConstraintComponent* Constraint = FastenerGridRef->PhysicsConstraints[Entry.FastenerIndex];
			if (!Constraint) continue;

			FConstraintInstance Profile = Constraint->ConstraintInstance;

			Profile.LinearDriveZDrive.Stiffness = Entry.ReducedVerticalStiffness;
			Profile.LinearDriveYDrive.Stiffness = Entry.ReducedLateralStiffness;

			float OriginalDampZ = FastenerGridRef->StiffnessParams.VerticalDamping;
			float OriginalDampY = FastenerGridRef->StiffnessParams.LateralDamping;
			Profile.LinearDriveZDrive.Damping = OriginalDampZ * Entry.StiffnessReductionFactor;
			Profile.LinearDriveYDrive.Damping = OriginalDampY * Entry.StiffnessReductionFactor;

			Constraint->ConstraintInstance = Profile;
			Updated++;
		}
	}

	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Propagated stiffness to %d fastener constraints"), Updated);
}

void ASubsidenceOperator::UpdateTrackMeshVertices()
{
	if (!TrackGeneratorRef) return;

	CacheOriginalVertices();

	if (TrackGeneratorRef->LeftRailMesh)
	{
		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		TArray<FVector2D> UVs;
		TArray<FLinearColor> Colors;
		TArray<FProcMeshTangent> Tangents;

		TrackGeneratorRef->LeftRailMesh->GetMeshSection(0, Vertices, Triangles, Normals, UVs, Colors, Tangents);

		for (int32 i = 0; i < Vertices.Num(); ++i)
		{
			float Settlement = GetSettlementAtDistance(Vertices[i].X);
			if (FMath::Abs(Settlement) > KINDA_SMALL_NUMBER)
			{
				Vertices[i].Z += Settlement;
			}
		}

		TrackGeneratorRef->LeftRailMesh->UpdateMeshSection(0, Vertices, Normals, UVs, Colors, Tangents);
	}

	if (TrackGeneratorRef->RightRailMesh)
	{
		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		TArray<FVector2D> UVs;
		TArray<FLinearColor> Colors;
		TArray<FProcMeshTangent> Tangents;

		TrackGeneratorRef->RightRailMesh->GetMeshSection(1, Vertices, Triangles, Normals, UVs, Colors, Tangents);

		for (int32 i = 0; i < Vertices.Num(); ++i)
		{
			float Settlement = GetSettlementAtDistance(Vertices[i].X);
			if (FMath::Abs(Settlement) > KINDA_SMALL_NUMBER)
			{
				Vertices[i].Z += Settlement;
			}
		}

		TrackGeneratorRef->RightRailMesh->UpdateMeshSection(1, Vertices, Normals, UVs, Colors, Tangents);
	}

	if (TrackGeneratorRef->SleeperMesh)
	{
		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		TArray<FVector2D> UVs;
		TArray<FLinearColor> Colors;
		TArray<FProcMeshTangent> Tangents;

		TrackGeneratorRef->SleeperMesh->GetMeshSection(2, Vertices, Triangles, Normals, UVs, Colors, Tangents);

		for (int32 i = 0; i < Vertices.Num(); ++i)
		{
			float Settlement = GetSettlementAtDistance(Vertices[i].X);
			if (FMath::Abs(Settlement) > KINDA_SMALL_NUMBER)
			{
				Vertices[i].Z += Settlement;
			}
		}

		TrackGeneratorRef->SleeperMesh->UpdateMeshSection(2, Vertices, Normals, UVs, Colors, Tangents);
	}

	if (TrackGeneratorRef->BallastMesh)
	{
		TArray<FVector> Vertices;
		TArray<int32> Triangles;
		TArray<FVector> Normals;
		TArray<FVector2D> UVs;
		TArray<FLinearColor> Colors;
		TArray<FProcMeshTangent> Tangents;

		TrackGeneratorRef->BallastMesh->GetMeshSection(3, Vertices, Triangles, Normals, UVs, Colors, Tangents);

		for (int32 i = 0; i < Vertices.Num(); ++i)
		{
			float Settlement = GetSettlementAtDistance(Vertices[i].X);
			if (FMath::Abs(Settlement) > KINDA_SMALL_NUMBER)
			{
				Vertices[i].Z += Settlement;
			}
		}

		TrackGeneratorRef->BallastMesh->UpdateMeshSection(3, Vertices, Normals, UVs, Colors, Tangents);
	}
}

float ASubsidenceOperator::GetSettlementAtDistance(float DistanceAlongTrack) const
{
	float Total = 0.0f;
	for (const auto& Zone : SubsidenceZones)
	{
		if (Zone.bActive)
		{
			Total += Zone.ComputeSettlement(DistanceAlongTrack);
		}
	}

	if (bProgressiveDegradation && FMath::Abs(Total) > KINDA_SMALL_NUMBER)
	{
		Total = ComputeTimeDependentSettlement(Total, CurrentServiceYears);
	}

	return Total;
}

float ASubsidenceOperator::GetSettlementSlopeAtDistance(float DistanceAlongTrack) const
{
	float Epsilon = 0.01f;
	float S0 = GetSettlementAtDistance(DistanceAlongTrack - Epsilon);
	float S1 = GetSettlementAtDistance(DistanceAlongTrack + Epsilon);
	return (S1 - S0) / (2.0f * Epsilon);
}

FStiffnessReductionEntry ASubsidenceOperator::GetFastenerReduction(int32 FastenerIndex) const
{
	if (StiffnessReductionMatrix.IsValidIndex(FastenerIndex))
	{
		return StiffnessReductionMatrix[FastenerIndex];
	}
	return FStiffnessReductionEntry();
}

float ASubsidenceOperator::GetEffectiveStiffness(int32 FastenerIndex, bool bVertical) const
{
	const auto& Entry = GetFastenerReduction(FastenerIndex);
	return bVertical ? Entry.ReducedVerticalStiffness : Entry.ReducedLateralStiffness;
}

float ASubsidenceOperator::GetEffectivePretension(int32 FastenerIndex) const
{
	return GetFastenerReduction(FastenerIndex).ReducedPretension;
}

void ASubsidenceOperator::SetServiceYears(float Years)
{
	CurrentServiceYears = Years;
	UpdateStiffnessReductionMatrix();
	PropagateStiffnessToFastenerGrid();
	UpdateTrackMeshVertices();

	UE_LOG(LogHSRTrackDynamics, Log,
		TEXT("Service years updated to %.1f, re-evaluating degradation"), Years);
}

float ASubsidenceOperator::ComputeTimeDependentSettlement(float BaseSettlement, float Years) const
{
	float TimeFactor = 1.0f + TimeHardeningFactor * FMath::Loge(1.0f + Years / DegradationYears);
	return BaseSettlement * TimeFactor;
}

float ASubsidenceOperator::ComputeTimeDependentStiffness(float BaseReduction, float Years) const
{
	float TimeFactor = 1.0f + 0.5f * (1.0f - FMath::Exp(-Years / DegradationYears));
	return FMath::Min(BaseReduction * TimeFactor, MaxStiffnessReduction);
}

void ASubsidenceOperator::ResetAllSettlements()
{
	SubsidenceZones.Empty();
	StiffnessReductionMatrix.Empty();
	TotalAffectedFasteners = 0;
	MaxSettlementDepth = 0.0f;
	MaxSettlementSlope = 0.0f;
	MaxStiffnessReductionApplied = 0.0f;
	MinPretensionRemaining = 1.0f;

	if (TrackGeneratorRef)
	{
		TrackGeneratorRef->GenerateTrack();
	}

	UE_LOG(LogHSRTrackDynamics, Log, TEXT("All settlements reset, track regenerated"));
}

void ASubsidenceOperator::CacheOriginalVertices()
{
	bOriginalVerticesCached = true;
}

float ASubsidenceOperator::ComputeCurveValue(float NormalizedDist, ESubsidenceCurveType CurveType) const
{
	if (NormalizedDist < 0.0f || NormalizedDist > 1.0f) return 0.0f;

	switch (CurveType)
	{
	case ESubsidenceCurveType::HalfSine:
		return FMath::Sin(NormalizedDist * PI);
	case ESubsidenceCurveType::CubicPolynomial:
		return 3.0f * NormalizedDist * NormalizedDist - 2.0f * NormalizedDist * NormalizedDist * NormalizedDist;
	case ESubsidenceCurveType::QuinticPolynomial:
		return 10.0f * FMath::Pow(NormalizedDist, 3) - 15.0f * FMath::Pow(NormalizedDist, 4) + 6.0f * FMath::Pow(NormalizedDist, 5);
	case ESubsidenceCurveType::Gaussian:
		{
			float Sigma = 0.2f;
			return FMath::Exp(-0.5f * FMath::Pow((NormalizedDist - 0.5f) / Sigma, 2));
		}
	case ESubsidenceCurveType::CosineTaper:
		return 0.5f * (1.0f - FMath::Cos(2.0f * PI * NormalizedDist));
	default:
		return FMath::Sin(NormalizedDist * PI);
	}
}

float ASubsidenceOperator::ComputeCurveDerivative(float NormalizedDist, ESubsidenceCurveType CurveType) const
{
	float Epsilon = 0.001f;
	float V0 = ComputeCurveValue(FMath::Clamp(NormalizedDist - Epsilon, 0.0f, 1.0f), CurveType);
	float V1 = ComputeCurveValue(FMath::Clamp(NormalizedDist + Epsilon, 0.0f, 1.0f), CurveType);
	return (V1 - V0) / (2.0f * Epsilon);
}
