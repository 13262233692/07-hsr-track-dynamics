#pragma once

#include "CoreMinimal.h"
#include "HSRTrackDynamics.h"
#include "NumericalStabilizer.generated.h"

UENUM(BlueprintType)
enum class EStabilityCondition : uint8
{
	Stable UMETA(DisplayName = "Stable"),
	Warning UMETA(DisplayName = "Warning"),
	Critical UMETA(DisplayName = "Critical"),
	Unstable UMETA(DisplayName = "Unstable")
};

USTRUCT(BlueprintType)
struct FStabilityAnalysisResult
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	EStabilityCondition Condition = EStabilityCondition::Stable;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float StabilityRatio = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxAllowedDt = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float CurrentDt = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float HighestFrequency = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float StepsPerPeriod = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float NumericalDampingRatio = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	bool bViolatesCourant = false;
};

USTRUCT(BlueprintType)
struct FStiffnessScalingConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Courant")
	float MinStepsPerPeriod = 20.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Courant")
	float SafetyFactor = 0.7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Damping")
	float TargetDampingRatio = 0.05f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Damping")
	float MinDampingRatio = 0.02f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Damping")
	float MaxDampingRatio = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Energy")
	bool bEnableEnergyPreservation = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Energy")
	float MaxEnergyGainPerStep = 0.001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Rayleigh")
	float RayleighAlphaMass = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Rayleigh")
	float RayleighBetaStiffness = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Auto")
	bool bAutoScaleDamping = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Auto")
	bool bAutoScaleStiffness = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Auto")
	float StiffnessScaleCeiling = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scaling|Auto")
	float StiffnessScaleFloor = 0.1f;
};

USTRUCT(BlueprintType)
struct FDamperBindingConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Primary")
	float PrimaryVerticalDamping = 4.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Primary")
	float PrimaryLateralDamping = 2.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Secondary")
	float SecondaryVerticalDamping = 3.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Secondary")
	float SecondaryLateralDamping = 1.5e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Rail")
	float RailFastenerDamping = 5.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|WheelRail")
	float WheelRailContinuousDamping = 1.0e3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Adaptive")
	bool bAdaptiveDamper = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Adaptive")
	float DampingScaleAtHighFreq = 2.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Adaptive")
	float HighFrequencyThreshold = 500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Nonlinear")
	bool bNonlinearDamper = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Nonlinear")
	float DamperExponent = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Damper|Nonlinear")
	float VelocityForMaxDamping = 1.0f;
};

UCLASS(ClassGroup = (HSRTrackDynamics), meta = (BlueprintSpawnableComponent))
class HSRTRACKDYNAMICS_API UNumericalStabilizer : public UActorComponent
{
	GENERATED_BODY()

public:
	UNumericalStabilizer();

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stabilizer|Config")
	FStiffnessScalingConfig StiffnessConfig;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Stabilizer|Config")
	FDamperBindingConfig DamperConfig;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Stabilizer|State")
	FStabilityAnalysisResult StabilityStatus;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Stabilizer|State")
	float CurrentStiffnessScale = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Stabilizer|State")
	float CurrentDampingScale = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Stabilizer|State")
	float TotalMechanicalEnergy = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Stabilizer|State")
	float EnergyTrend = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Stabilizer|State")
	int32 StabilityViolationCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Stabilizer|State")
	int32 EmergencyCorrections = 0;

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Analysis")
	FStabilityAnalysisResult AnalyzeStability(
		float DominantFrequency,
		float DominantStiffness,
		float DominantMass,
		float CurrentTimeStep);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Compute")
	float ComputeOptimalTimeStep(
		float DominantFrequency,
		float DominantStiffness,
		float DominantMass);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Compute")
	float ComputeRayleighDamping(
		float AngularFrequency,
		float Mass,
		float Stiffness);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Compute")
	float ComputeNumericalDampingRatio(
		float TimeStep,
		float AngularFrequency,
		float DampingRatio);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Compute")
	float ComputeNonlinearDamperForce(
		float Velocity,
		float BaseDamping,
		float ScaleFactor = 1.0f);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Compute")
	FVector ComputeNonlinearDamperForce3D(
		const FVector& Velocity,
		const FVector& BaseDamping,
		float ScaleFactor = 1.0f);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Energy")
	void UpdateEnergyBalance(
		float KineticEnergy,
		float PotentialEnergy,
		float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Energy")
	float GetTotalEnergy() const { return TotalMechanicalEnergy; }

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Correction")
	float ApplyEnergyCorrection(
		float Velocity,
		float Mass,
		float MaxEnergyGain);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Correction")
	FVector ApplyEnergyCorrection3D(
		const FVector& Velocity,
		float Mass,
		float MaxEnergyGain);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Update")
	void UpdateScalingFactors(float DeltaTime, float DominantFrequency);

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Reset")
	void ResetStabilizer();

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Check")
	bool CheckCourantFriedrichsLewy(
		float WaveSpeed,
		float ElementSize,
		float TimeStep) const;

	UFUNCTION(BlueprintCallable, Category = "Stabilizer|Check")
	bool CheckNewmarkStability(
		float Gamma,
		float Beta,
		float DampingRatio) const;

private:
	float PreviousEnergy = 0.0f;
	float EnergyHistory[8] = { 0.0f };
	int32 EnergyHistoryIndex = 0;

	float ComputeEffectiveFrequency(float AngularFreq, float DampingRatio) const;
	float SmoothStep(float Edge0, float Edge1, float X) const;
};
