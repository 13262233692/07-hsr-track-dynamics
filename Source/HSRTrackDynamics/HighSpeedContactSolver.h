#pragma once

#include "CoreMinimal.h"
#include "RK4Integrator.h"
#include "WheelRailContact.h"
#include "HighSpeedContactSolver.generated.h"

USTRUCT(BlueprintType)
struct FContactSubstepDiagnostics
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 SubstepCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MinSubstepDt = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxContactForce = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxPenetration = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxPressure = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 ContactIterations = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float EnergyDissipated = 0.0f;
};

USTRUCT(BlueprintType)
struct FHighSpeedContactParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Substep")
	float BaseSubstepTime = 1.0e-5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Substep")
	float MinSubstepTime = 1.0e-7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Substep")
	int32 MaxSubstepsPerFrame = 1000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Substep")
	float PenetrationTolerance = 1.0e-7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Iteration")
	int32 MaxContactIterations = 20;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Iteration")
	float ContactForceTolerance = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Stability")
	float NumericalDampingCoeff = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Stability")
	bool bUseAdaptiveSubstepping = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Stability")
	float ImpactVelocityThreshold = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Stability")
	bool bUsePenaltyMethod = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Stability")
	float PenaltyStiffnessMultiplier = 1.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Hertz")
	float HertzExponent = 1.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "HighSpeed|Hertz")
	bool bNonlinearHertzContact = true;
};

UCLASS(ClassGroup = (HSRTrackDynamics), meta = (BlueprintSpawnableComponent))
class HSRTRACKDYNAMICS_API UHighSpeedContactSolver : public UActorComponent
{
	GENERATED_BODY()

public:
	UHighSpeedContactSolver();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Params")
	FHighSpeedContactParams SolverParams;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Wheel")
	float WheelRadius = 0.46f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Wheel")
	float WheelsetMass = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Wheel")
	float WheelsetInertiaY = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Wheel")
	float WheelsetInertiaXZ = 700.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Rail")
	float RailStiffnessVertical = 6.0e7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Rail")
	float RailDampingVertical = 5.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Rail")
	float CombinedElasticModulus = 1.15e11f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Input")
	FVector InitialWheelPosition = FVector(0.0f, 0.0f, 0.46f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Input")
	FVector InitialWheelVelocity = FVector(83.33f, 0.0f, 0.0f);

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Input")
	float InitialWheelSpinVelocity = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Input")
	float RailSurfaceHeight = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Input")
	float RailCurvatureRadius = 3000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Input")
	float RailIrregularityAmplitude = 0.001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Input")
	float RailIrregularityWavelength = 0.5f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Solver|State")
	FRK4WheelsetState WheelsetState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Solver|State")
	FContactPatchResult ContactResult;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Solver|Diagnostics")
	FContactSubstepDiagnostics SubstepDiagnostics;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Solver|State")
	float TotalSimulatedTime = 0.0f;

	UFUNCTION(BlueprintCallable, Category = "Solver|Control")
	void InitializeSolver();

	UFUNCTION(BlueprintCallable, Category = "Solver|Control")
	void ResetSolver();

	UFUNCTION(BlueprintCallable, Category = "Solver|Advance")
	void AdvanceSimulation(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Solver|Force")
	FVector ComputeContactForce(const FRK4WheelsetState& State, float Time);

	UFUNCTION(BlueprintCallable, Category = "Solver|Force")
	float ComputeHertzNormalForce(float Penetration);

	UFUNCTION(BlueprintCallable, Category = "Solver|Force")
	float ComputeHertzPenetration(float NormalForce);

	UFUNCTION(BlueprintCallable, Category = "Solver|Force")
	FVector ComputeTangentialCreepForces(float NormalForce, const FRK4WheelsetState& State);

	UFUNCTION(BlueprintCallable, Category = "Solver|Geometry")
	float GetRailHeightAt(float LongitudinalPos, float LateralPos) const;

	UFUNCTION(BlueprintCallable, Category = "Solver|Geometry")
	FVector GetRailNormalAt(float LongitudinalPos, float LateralPos) const;

private:
	FRK4WheelsetDerivative ComputeWheelsetDerivative(
		const FRK4WheelsetState& State, float Time);

	void PerformContactIteration(
		FRK4WheelsetState& State, float Time, float Dt, int32 Iteration);

	void ApplyNumericalDamping(FRK4WheelsetState& State, float Dt);

	float ComputeAdaptiveSubstep(float CurrentDt, const FRK4WheelsetState& State);

	float ComputeEffectiveContactRadius(float LateralShift) const;
};
