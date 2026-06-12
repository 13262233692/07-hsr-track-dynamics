#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "TrackGenerator.h"
#include "FastenerSpringGrid.h"
#include "TrainMBSVehicle.h"
#include "TrackDynamicsSolver.generated.h"

USTRUCT(BlueprintType)
struct FRailIrregularitySpectrum
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	bool bEnabled = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	float AWavelengthA = 4.032e-3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	float AWavelengthB = 1.467e-6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	float CWavelengthA = 5.32e-3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	float CWavelengthB = 6.34e-7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	int32 SpectrumSamples = 256;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	float MaxWavelength = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	float MinWavelength = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Irregularity")
	int32 RandomSeed = 42;
};

USTRUCT(BlueprintType)
struct FSimulationDiagnostics
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float CurrentSimTime = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float RealTimeFactor = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 SubstepCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxWheelForce = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxRailDisplacement = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxContactPressure = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float TotalEnergyDissipated = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float CurrentVehicleSpeed = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxLateralCreepage = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 ActiveFastenerCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float CurrentStiffnessScale = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float CurrentDampingScale = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float DominantFrequency = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float StabilityRatio = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 TotalRK4Substeps = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 EnergyCorrections = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	int32 VelocityClamps = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	EStabilityCondition StabilityCondition = EStabilityCondition::Stable;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float DerailmentCoefficient = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LoadReductionRatio = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float PeakVerticalAccel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float PeakCarbodyVertAccel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	bool bDerailmentImminent = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxSettlementDepth = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float MaxStiffnessReduction = 0.0f;
};

UCLASS()
class HSRTRACKDYNAMICS_API ATrackDynamicsSolver : public AActor
{
	GENERATED_BODY()

public:
	ATrackDynamicsSolver();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Refs")
	ATrackGenerator* TrackGeneratorRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Refs")
	AFastenerSpringGrid* FastenerGridRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Refs")
	ATrainMBSVehicle* TrainVehicleRef;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Refs")
	class ASubsidenceOperator* SubsidenceOperatorRef;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Solver|Components")
	class UDerailmentAssessor* DerailmentAssessor;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Timing")
	float PhysicsSubstepSize = 0.001f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Timing")
	int32 MaxSubstepsPerFrame = 16;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Timing")
	bool bUseFixedSubstep = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Timing")
	float SolverDampingFactor = 0.999f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Mode")
	bool bRunCoupledSimulation = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Mode")
	bool bEnableTrackDynamics = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Mode")
	bool bEnableVehicleDynamics = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Mode")
	bool bEnableIrregularity = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	bool bUseHighSpeedRK4Mode = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	bool bAutoDisableChaosPhysics = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	float RK4BaseSubstep = 1.0e-5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	float RK4MinSubstep = 1.0e-7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	int32 RK4MaxSubstepsPerFrame = 2000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	bool bAdaptiveSubstep = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	float ContactStiffnessScaling = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	float StabilitySafetyFactor = 0.7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|HighSpeed")
	float MinStepsPerPeriod = 30.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Stability")
	bool bEnableStabilityAnalysis = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Stability")
	bool bEnableEnergyCorrection = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Stability")
	float MaxEnergyGainPerStep = 0.005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Stability")
	float SpeedLimitKmh = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Solver|Irregularity")
	FRailIrregularitySpectrum IrregularitySpectrum;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Solver|Components")
	UNumericalStabilizer* NumericalStabilizer;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Solver|Diagnostics")
	FSimulationDiagnostics Diagnostics;

	UFUNCTION(BlueprintCallable, Category = "Solver|Control")
	void InitializeSimulation();

	UFUNCTION(BlueprintCallable, Category = "Solver|Control")
	void StartSimulation();

	UFUNCTION(BlueprintCallable, Category = "Solver|Control")
	void StopSimulation();

	UFUNCTION(BlueprintCallable, Category = "Solver|Control")
	void ResetSimulation();

	UFUNCTION(BlueprintCallable, Category = "Solver|Physics")
	void ExecutePhysicsSubstep(float Dt);

	UFUNCTION(BlueprintCallable, Category = "Solver|Physics")
	void CoupleWheelToTrack();

	UFUNCTION(BlueprintCallable, Category = "Solver|Physics")
	void GenerateRailIrregularity();

	UFUNCTION(BlueprintCallable, Category = "Solver|Query")
	float GetRailIrregularityAt(float DistanceAlongTrack, bool bVertical) const;

	UFUNCTION(BlueprintCallable, Category = "Solver|Query")
	void GetSimulationStats(FSimulationDiagnostics& OutStats) const;

private:
	bool bSimulationRunning = false;

	TArray<float> VerticalIrregularitySpectrumMag;
	TArray<float> LateralIrregularitySpectrumMag;
	TArray<float> VerticalIrregularityPhase;
	TArray<float> LateralIrregularityPhase;

	float AccumulatedTime = 0.0f;

	void GenerateIrregularitySpectrum();
	void UpdateDiagnostics(float DeltaTime);
	void LogDiagnostics();

	FRandomStream RandomStream;
};
