#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "DerailmentAssessor.generated.h"

UENUM(BlueprintType)
enum class EDerailmentRiskLevel : uint8
{
	Safe UMETA(DisplayName = "Safe"),
	Caution UMETA(DisplayName = "Caution"),
	Warning UMETA(DisplayName = "Warning"),
	Danger UMETA(DisplayName = "Danger"),
	Critical UMETA(DisplayName = "CRITICAL DERAILMENT")
};

USTRUCT(BlueprintType)
struct FWheelDerailmentData
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LateralForceQ = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VerticalForceP = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float DerailmentCoefficientQP = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float StaticWheelLoadP0 = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float WheelLoadReductionDeltaP = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LoadReductionRatio = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VerticalAcceleration = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LateralAcceleration = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	EDerailmentRiskLevel QPRiskLevel = EDerailmentRiskLevel::Safe;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	EDerailmentRiskLevel LoadReductionRiskLevel = EDerailmentRiskLevel::Safe;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	EDerailmentRiskLevel OverallRiskLevel = EDerailmentRiskLevel::Safe;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float NadalLimit = 0.8f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	bool bExceedsNadal = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	bool bExceedsLoadReduction = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float FlangeContactAngle = 70.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float FrictionCoefficient = 0.35f;
};

USTRUCT(BlueprintType)
struct FDerailmentAssessmentReport
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	TArray<FWheelDerailmentData> WheelData;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxDerailmentCoefficient = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxLoadReductionRatio = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float PeakVerticalAcceleration = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float PeakLateralAcceleration = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	EDerailmentRiskLevel OverallRiskLevel = EDerailmentRiskLevel::Safe;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 WorstWheelIndex = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VehicleSpeed = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float TrackPosition = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	bool bDerailmentImminent = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float TimeInWarningZone = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxCarbodyVerticalAccel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxCarbodyLateralAccel = 0.0f;
};

UCLASS(ClassGroup = (HSRTrackDynamics), meta = (BlueprintSpawnableComponent))
class HSRTRACKDYNAMICS_API UDerailmentAssessor : public UActorComponent
{
	GENERATED_BODY()

public:
	UDerailmentAssessor();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Thresholds")
	float NadalCoefficientLimit = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Thresholds")
	float CautionQPRatio = 0.6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Thresholds")
	float WarningQPRatio = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Thresholds")
	float DangerQPRatio = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Thresholds")
	float LoadReductionCaution = 0.4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Thresholds")
	float LoadReductionWarning = 0.6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Thresholds")
	float LoadReductionDanger = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Acceleration")
	float VerticalAccelCaution = 2.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Acceleration")
	float VerticalAccelWarning = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Acceleration")
	float VerticalAccelDanger = 8.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Acceleration")
	float LateralAccelCaution = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Acceleration")
	float LateralAccelWarning = 3.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Acceleration")
	float LateralAccelDanger = 5.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Nadal")
	float FlangeAngleDeg = 70.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Nadal")
	float WheelRailFriction = 0.35f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Filter")
	float AccelFilterCutoff = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Filter")
	bool bEnableLowPassFilter = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	TArray<float> WheelLateralForces;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	TArray<float> WheelVerticalForces;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	TArray<float> WheelVerticalAccelerations;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	TArray<float> WheelLateralAccelerations;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	float CarbodyVerticalAcceleration = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	float CarbodyLateralAcceleration = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	float CurrentSpeed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	float StaticWheelLoad = 8500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Derailment|Input")
	int32 NumWheels = 8;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Derailment|Report")
	FDerailmentAssessmentReport CurrentReport;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Derailment|Report")
	float TimeSinceLastWarning = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Derailment|Report")
	int32 WarningCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Derailment|Report")
	int32 DangerCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Derailment|Report")
	float PeakVerticalAccelHistory = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Derailment|Report")
	float PeakLateralAccelHistory = 0.0f;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Compute")
	void ComputeDerailmentAssessment();

	UFUNCTION(BlueprintCallable, Category = "Derailment|Compute")
	float ComputeNadalLimit() const;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Compute")
	float ComputeDerailmentCoefficient(float LateralForceQ, float VerticalForceP) const;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Compute")
	float ComputeLoadReductionRatio(float CurrentVerticalForce, float StaticLoad) const;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Compute")
	EDerailmentRiskLevel AssessQPRisk(float QP) const;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Compute")
	EDerailmentRiskLevel AssessLoadReductionRisk(float Ratio) const;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Compute")
	EDerailmentRiskLevel AssessAccelerationRisk(float VertAccel, float LatAccel) const;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Filter")
	float ApplyLowPassFilter(float RawValue, float PreviousFiltered, float CutoffHz, float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Derailment|Report")
	FString GetRiskLevelString(EDerailmentRiskLevel Level) const;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Report")
	FString GetSummaryString() const;

	UFUNCTION(BlueprintCallable, Category = "Derailment|Reset")
	void ResetAssessor();

private:
	TArray<float> FilteredVertAccels;
	TArray<float> FilteredLatAccels;
	float FilteredCarbodyVert = 0.0f;
	float FilteredCarbodyLat = 0.0f;

	EDerailmentRiskLevel GetWorstRisk(EDerailmentRiskLevel A, EDerailmentRiskLevel B) const;
};
