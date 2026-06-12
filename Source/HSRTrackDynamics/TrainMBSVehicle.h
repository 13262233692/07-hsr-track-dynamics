#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "WheelRailContact.h"
#include "RK4Integrator.h"
#include "NumericalStabilizer.h"
#include "HighSpeedContactSolver.h"
#include "DerailmentAssessor.h"
#include "TrainMBSVehicle.generated.h"

USTRUCT(BlueprintType)
struct FPrimarySuspensionParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Primary")
	float AxleBoxSpringStiffnessZ = 1.2e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Primary")
	float AxleBoxSpringStiffnessY = 5.0e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Primary")
	float AxleBoxSpringStiffnessX = 1.0e7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Primary")
	float AxleBoxDampingZ = 4.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Primary")
	float AxleBoxDampingY = 2.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Primary")
	float AxleBoxDampingX = 3.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Primary")
	float PrimaryVerticalStiffnessRoll = 5.0e5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Primary")
	float PrimaryDampingRoll = 1.0e3f;
};

USTRUCT(BlueprintType)
struct FSecondarySuspensionParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringStiffnessZ = 4.0e5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringStiffnessY = 2.0e5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringStiffnessX = 3.0e5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringDampingZ = 3.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringDampingY = 1.5e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringDampingX = 2.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float SecondaryRollStiffness = 2.0e5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float SecondaryRollDamping = 5.0e3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float LateralBumperStiffness = 1.0e7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float LateralBumperGap = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringAuxVolume = 0.06f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringEffectiveArea = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringPolytropicIndex = 1.38f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	float AirSpringInitialPressure = 5.0e5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Suspension|Secondary")
	bool bNonlinearAirSpring = true;
};

USTRUCT(BlueprintType)
struct FWheelsetDofState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LateralDisp = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VerticalDisp = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float RollAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float YawAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LateralVel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VerticalVel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float RollRate = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float YawRate = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float WheelAngularVel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LongitudinalPos = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LongitudinalVel = 0.0f;
};

USTRUCT(BlueprintType)
struct FBogieDofState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LateralDisp = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VerticalDisp = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float RollAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float PitchAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float YawAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LateralVel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VerticalVel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float RollRate = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float PitchRate = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float YawRate = 0.0f;
};

USTRUCT(BlueprintType)
struct FCarBodyDofState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LateralDisp = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VerticalDisp = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float RollAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float PitchAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float YawAngle = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LateralVel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float VerticalVel = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float RollRate = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float PitchRate = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float YawRate = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	float LongitudinalVel = 0.0f;
};

UCLASS()
class HSRTRACKDYNAMICS_API ATrainMBSVehicle : public AActor
{
	GENERATED_BODY()

public:
	ATrainMBSVehicle();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float CarBodyMass = 40000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float CarBodyInertiaXX = 5.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float CarBodyInertiaYY = 2.0e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float CarBodyInertiaZZ = 2.0e6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float BogieFrameMass = 2500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float BogieFrameInertiaXX = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float BogieFrameInertiaYY = 2000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float BogieFrameInertiaZZ = 2500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float WheelsetMass = 1500.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float WheelsetInertiaXX = 700.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float WheelsetInertiaYY = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Inertia")
	float WheelsetInertiaZZ = 700.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Geometry")
	float BogieHalfSpacing = 8.75f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Geometry")
	float AxleHalfSpacing = 1.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Geometry")
	float Gauge = 1.435f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Geometry")
	float WheelRadius = 0.46f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Geometry")
	float PrimaryVerticalOffset = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Geometry")
	float SecondaryVerticalOffset = 0.8f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Suspension")
	FPrimarySuspensionParams PrimarySuspension;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Suspension")
	FSecondarySuspensionParams SecondarySuspension;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Operation")
	float TargetSpeed = 83.33f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Operation")
	float AxleLoad = 17000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Operation")
	bool bEnableTractionControl = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Operation")
	float TractionGain = 5.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Operation")
	float TractionDerivativeGain = 1.0e3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	bool bUseRK4Integration = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	bool bDisableChaosPhysics = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	float RK4BaseSubstep = 1.0e-5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	float RK4MinSubstep = 1.0e-7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	int32 RK4MaxSubstepsPerFrame = 2000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	bool bAdaptiveSubstepping = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	float ImpactVelocityThreshold = 0.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	float WheelRailContactStiffness = 1.0e9f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	float SpeedLimitKmh = 400.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	bool bEnableVelocityClamping = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|HighSpeed")
	float MaxAngularVelocity = 100.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Energy")
	bool bEnableEnergyMonitor = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Energy")
	float MaxEnergyGainPerSubstep = 0.005f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Energy")
	float EmergencyDampingFactor = 0.9f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	TArray<FWheelsetDofState> WheelsetStates;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	TArray<FBogieDofState> BogieStates;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	FCarBodyDofState CarBodyState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	TArray<FContactPatchResult> WheelContactResults;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	TArray<FRK4WheelsetState> RK4WheelsetStates;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	FRK4CarBodyState RK4CarBodyState;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	float TotalMechanicalEnergy = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	float DominantFrequency = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	int32 TotalSubstepsThisFrame = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	int32 VelocityClampCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	int32 EnergyCorrectionCount = 0;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	float CurrentStiffnessScale = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|State")
	float CurrentDampingScale = 1.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	TArray<UStaticMeshComponent*> WheelsetBodies;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	TArray<UStaticMeshComponent*> BogieFrameBodies;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	UStaticMeshComponent* CarBodyMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	TArray<UPhysicsConstraintComponent*> PrimaryConstraints;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	TArray<UPhysicsConstraintComponent*> SecondaryConstraints;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	TArray<UWheelRailContact*> WheelRailContacts;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	UNumericalStabilizer* NumericalStabilizer;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	UHighSpeedContactSolver* HighSpeedContactSolver;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "MBS|Components")
	class UDerailmentAssessor* DerailmentAssessor;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "MBS|Refs")
	class ASubsidenceOperator* SubsidenceOperatorRef;

	UFUNCTION(BlueprintCallable, Category = "MBS|Setup")
	void InitializeVehicle();

	UFUNCTION(BlueprintCallable, Category = "MBS|Physics")
	void StepMBSDynamics(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "MBS|HighSpeed")
	void StepRK4Dynamics(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "MBS|HighSpeed")
	void InitializeRK4States();

	UFUNCTION(BlueprintCallable, Category = "MBS|HighSpeed")
	FRK4WheelsetDerivative ComputeWheelsetRK4Derivative(
		const FRK4WheelsetState& State, float Time, int32 WheelsetIdx);

	UFUNCTION(BlueprintCallable, Category = "MBS|HighSpeed")
	FRK4CarBodyDerivative ComputeCarBodyRK4Derivative(
		const FRK4CarBodyState& State, float Time);

	UFUNCTION(BlueprintCallable, Category = "MBS|HighSpeed")
	void ComputeSuspensionForcesRK4(
		int32 WheelsetIdx,
		const FRK4WheelsetState& WSState,
		float Time,
		FVector& OutLinearForce,
		FVector& OutTorque);

	UFUNCTION(BlueprintCallable, Category = "MBS|HighSpeed")
	float ComputeAdaptiveSubstepSize(
		float BaseDt,
		const FRK4WheelsetState& State) const;

	UFUNCTION(BlueprintCallable, Category = "MBS|Energy")
	float ComputeTotalMechanicalEnergy() const;

	UFUNCTION(BlueprintCallable, Category = "MBS|Energy")
	float ComputeWheelsetKineticEnergy(int32 WheelsetIdx) const;

	UFUNCTION(BlueprintCallable, Category = "MBS|Energy")
	float ComputeCarBodyKineticEnergy() const;

	UFUNCTION(BlueprintCallable, Category = "MBS|Energy")
	float ComputePotentialEnergy() const;

	UFUNCTION(BlueprintCallable, Category = "MBS|Safety")
	void ClampVelocitiesToLimits();

	UFUNCTION(BlueprintCallable, Category = "MBS|Safety")
	void ApplyEnergyCorrection();

	UFUNCTION(BlueprintCallable, Category = "MBS|Safety")
	bool CheckStabilityCondition();

	UFUNCTION(BlueprintCallable, Category = "MBS|Physics")
	void ComputeWheelRailForces();

	UFUNCTION(BlueprintCallable, Category = "MBS|Physics")
	void ComputePrimarySuspensionForces(int32 WheelsetIdx, FVector& OutForce, FVector& OutTorque);

	UFUNCTION(BlueprintCallable, Category = "MBS|Physics")
	void ComputeSecondarySuspensionForces(int32 BogieIdx, FVector& OutForce, FVector& OutTorque);

	UFUNCTION(BlueprintCallable, Category = "MBS|Physics")
	float ComputeAirSpringForce(float Displacement, float Velocity, int32 BogieIdx);

	UFUNCTION(BlueprintCallable, Category = "MBS|Physics")
	void ApplyTractionControl(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "MBS|Query")
	FVector GetWheelContactPosition(int32 WheelsetIdx, bool bLeftWheel) const;

	UFUNCTION(BlueprintCallable, Category = "MBS|Query")
	float GetCurrentSpeed() const;

private:
	static constexpr int32 NUM_BOGIES = 2;
	static constexpr int32 NUM_WHEELSETS_PER_BOGIE = 2;
	static constexpr int32 NUM_WHEELSETS = 4;
	static constexpr int32 NUM_WHEELS = 8;

	float SpeedErrorIntegral = 0.0f;

	void CreatePhysicsBodies();
	void CreateSuspensionConstraints();
	void IntegrateWheelsetState(int32 Idx, float Dt);
	void IntegrateBogieState(int32 Idx, float Dt);
	void IntegrateCarBodyState(float Dt);
	FVector ComputeLateralBumperForce(float LateralDisp, float LateralVel) const;
};
