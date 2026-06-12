#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SubsidenceOperator.generated.h"

UENUM(BlueprintType)
enum class ESubsidenceCurveType : uint8
{
	HalfSine UMETA(DisplayName = "Half-Sine"),
	CubicPolynomial UMETA(DisplayName = "Cubic Polynomial"),
	QuinticPolynomial UMETA(DisplayName = "Quintic Polynomial"),
	Gaussian UMETA(DisplayName = "Gaussian"),
	Cosine UMETA(DisplayName = "Cosine Taper")
};

USTRUCT(BlueprintType)
struct FSubsidenceZone
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	float StartDistance = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	float EndDistance = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	float MaxDepth = -0.030f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	ESubsidenceCurveType CurveType = ESubsidenceCurveType::HalfSine;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	float TaperLength = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	bool bAffectLeftRail = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	bool bAffectRightRail = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	bool bAsymmetric = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone", meta = (EditCondition = "bAsymmetric"))
	float AsymmetryOffset = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone", meta = (EditCondition = "bAsymmetric"))
	float LeftDepthMultiplier = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone", meta = (EditCondition = "bAsymmetric"))
	float RightDepthMultiplier = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zone")
	float LateralTiltAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	bool bActive = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	float AffectedLength = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	int32 AffectedFastenerCount = 0;

	float ComputeSettlement(float DistanceAlongTrack) const;
};

USTRUCT(BlueprintType)
struct FStiffnessReductionEntry
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 FastenerIndex = -1;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float OriginalVerticalStiffness = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float ReducedVerticalStiffness = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float OriginalLateralStiffness = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float ReducedLateralStiffness = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float OriginalPretension = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float ReducedPretension = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float StiffnessReductionFactor = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float SettlementAtFastener = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float SettlementSlope = 0.0f;
};

UCLASS()
class HSRTRACKDYNAMICS_API ASubsidenceOperator : public AActor
{
	GENERATED_BODY()

public:
	ASubsidenceOperator();

	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Refs")
	class ATrackGenerator* TrackGeneratorRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Refs")
	class AFastenerSpringGrid* FastenerGridRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Zones")
	TArray<FSubsidenceZone> SubsidenceZones;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Stiffness")
	float StiffnessReductionPerMm = 0.02f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Stiffness")
	float MaxStiffnessReduction = 0.7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Stiffness")
	float SlopeReductionRate = 0.005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Pretension")
	float BasePretension = 1.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Pretension")
	float PretensionLossPerMm = 200.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Pretension")
	float MinPretensionRatio = 0.2f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Progressive")
	bool bProgressiveDegradation = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Progressive", meta = (EditCondition = "bProgressiveDegradation"))
	float DegradationYears = 10.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Progressive", meta = (EditCondition = "bProgressiveDegradation"))
	float CurrentServiceYears = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Subsidence|Progressive", meta = (EditCondition = "bProgressiveDegradation"))
	float TimeHardeningFactor = 0.3f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	TArray<FStiffnessReductionEntry> StiffnessReductionMatrix;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	int32 TotalAffectedFasteners = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	float MaxSettlementDepth = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	float MaxSettlementSlope = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	float MaxStiffnessReductionApplied = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Subsidence|State")
	float MinPretensionRemaining = 1.0f;

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Apply")
	void ApplySubsidenceZone(int32 ZoneIndex);

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Apply")
	void ApplyAllSubsidenceZones();

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Apply")
	void AddSubsidenceZone(float StartDist, float EndDist, float MaxDepthMm, ESubsidenceCurveType CurveType);

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Apply")
	void RemoveSubsidenceZone(int32 ZoneIndex);

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Apply")
	void ClearAllSubsidenceZones();

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Update")
	void UpdateStiffnessReductionMatrix();

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Update")
	void PropagateStiffnessToFastenerGrid();

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Update")
	void UpdateTrackMeshVertices();

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Query")
	float GetSettlementAtDistance(float DistanceAlongTrack) const;

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Query")
	float GetSettlementSlopeAtDistance(float DistanceAlongTrack) const;

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Query")
	FStiffnessReductionEntry GetFastenerReduction(int32 FastenerIndex) const;

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Query")
	float GetEffectiveStiffness(int32 FastenerIndex, bool bVertical) const;

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Query")
	float GetEffectivePretension(int32 FastenerIndex) const;

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Progressive")
	void SetServiceYears(float Years);

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Progressive")
	float ComputeTimeDependentSettlement(float BaseSettlement, float Years) const;

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Progressive")
	float ComputeTimeDependentStiffness(float BaseReduction, float Years) const;

	UFUNCTION(BlueprintCallable, Category = "Subsidence|Reset")
	void ResetAllSettlements();

private:
	void ModifyProceduralMeshSection(class UProceduralMeshComponent* MeshComp, int32 SectionIndex,
		const TArray<FVector>& OriginalVertices, const TArray<int32>& Triangles);

	TArray<TArray<FVector>> OriginalLeftRailVertices;
	TArray<TArray<FVector>> OriginalRightRailVertices;
	TArray<TArray<FVector>> OriginalSleeperVertices;
	TArray<TArray<FVector>> OriginalBallastVertices;

	bool bOriginalVerticesCached = false;

	void CacheOriginalVertices();
	float ComputeCurveValue(float NormalizedDist, ESubsidenceCurveType CurveType) const;
	float ComputeCurveDerivative(float NormalizedDist, ESubsidenceCurveType CurveType) const;
};
