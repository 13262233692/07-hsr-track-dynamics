#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "WheelRailContact.generated.h"

USTRUCT(BlueprintType)
struct FHertzianContactParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Material")
	float WheelElasticModulus = 2.1e11f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Material")
	float RailElasticModulus = 2.1e11f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Material")
	float WheelPoissonRatio = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Material")
	float RailPoissonRatio = 0.3f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Geometry")
	float WheelRadius = 0.46f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Geometry")
	float RailHeadRadius = 0.30f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Geometry")
	float WheelProfileCurvature = 1.0f / 0.46f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Geometry")
	float LateralRailCurvature = 1.0f / 0.30f;
};

USTRUCT(BlueprintType)
struct FKalkerCoefficients
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Creep|Kalker")
	float C11 = 4.12f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Creep|Kalker")
	float C22 = 3.67f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Creep|Kalker")
	float C23 = 1.47f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Creep|Kalker")
	float C33 = 0.545f;
};

USTRUCT(BlueprintType)
struct FContactPatchResult
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	float NormalForce = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	FVector TangentialForce = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	FVector ContactPointWorld = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	float ContactPatchSemiAxisA = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	float ContactPatchSemiAxisB = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	float MaxContactPressure = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	float PenetrationDepth = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	FVector LongitudinalCreepForce = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	FVector LateralCreepForce = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	float SpinCreepMoment = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|Result")
	bool bInContact = false;
};

USTRUCT(BlueprintType)
struct FCreepageState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Creep|State")
	float LongitudinalCreepage = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Creep|State")
	float LateralCreepage = 0.0f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Creep|State")
	float SpinCreepage = 0.0f;
};

UCLASS(ClassGroup = (HSRTrackDynamics), meta = (BlueprintSpawnableComponent))
class HSRTRACKDYNAMICS_API UWheelRailContact : public UActorComponent
{
	GENERATED_BODY()

public:
	UWheelRailContact();

	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Params")
	FHertzianContactParams HertzParams;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Params")
	FKalkerCoefficients KalkerCoeffs;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Params")
	float WheelLoad = 7.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Params")
	float StaticFrictionCoefficient = 0.35f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Params")
	float KineticFrictionCoefficient = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Params")
	float CreepageSaturationThreshold = 0.01f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Params")
	bool bEnableSpinCreep = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Params")
	bool bUseSimplifiedFASTSIM = false;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|State")
	FContactPatchResult LastContactResult;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Contact|State")
	FCreepageState CurrentCreepage;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Input")
	FVector WheelCenterPosition = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Input")
	FVector WheelVelocity = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Input")
	float WheelAngularVelocity = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Input")
	float WheelLateralDisplacement = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Input")
	FVector RailTopPosition = FVector::ZeroVector;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Input")
	float VehicleSpeed = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Input")
	float RailIrregularityVertical = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Contact|Input")
	float RailIrregularityLateral = 0.0f;

	UFUNCTION(BlueprintCallable, Category = "Contact|Calculation")
	FContactPatchResult ComputeHertzianContact(float NormalLoad);

	UFUNCTION(BlueprintCallable, Category = "Contact|Calculation")
	void ComputeCreepage();

	UFUNCTION(BlueprintCallable, Category = "Contact|Calculation")
	FVector ComputeKalkerLinearCreepForces(float NormalLoad, const FContactPatchResult& Patch);

	UFUNCTION(BlueprintCallable, Category = "Contact|Calculation")
	FVector ComputeSaturatedCreepForces(const FVector& LinearForce, float NormalLoad);

	UFUNCTION(BlueprintCallable, Category = "Contact|Calculation")
	FVector ComputeFASTSIMCreepForces(float NormalLoad, const FContactPatchResult& Patch);

	UFUNCTION(BlueprintCallable, Category = "Contact|Calculation")
	float ComputeEffectiveWheelRadius(float LateralShift) const;

	UFUNCTION(BlueprintCallable, Category = "Contact|Query")
	float GetRollingResistanceForce() const;

private:
	float ComputeCombinedCurvature() const;
	float ComputeEquivalentElasticModulus() const;
	float SolveHertzSemiAxes(float NormalLoad, float& OutA, float& OutB) const;
	void ApplyRailIrregularityToCreepage();

	static constexpr float PI = 3.14159265358979323846f;
};
