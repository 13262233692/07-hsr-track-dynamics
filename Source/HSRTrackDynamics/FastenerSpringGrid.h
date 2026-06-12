#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "FastenerSpringGrid.generated.h"

USTRUCT(BlueprintType)
struct FFastenerStiffnessParams
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Stiffness")
	float VerticalStiffness = 6.0e7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Stiffness")
	float VerticalDamping = 5.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Stiffness")
	float LateralStiffness = 4.0e7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Stiffness")
	float LateralDamping = 3.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Stiffness")
	float LongitudinalStiffness = 5.0e7f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Stiffness")
	float LongitudinalDamping = 4.0e4f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Stiffness")
	float RotationalStiffness = 1.0e5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Stiffness")
	float RotationalDamping = 5.0e2f;
};

USTRUCT(BlueprintType)
struct FFastenerNodeState
{
	GENERATED_BODY()

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fastener|State")
	FVector RailDisplacement = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fastener|State")
	FVector RailVelocity = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fastener|State")
	FVector ConstraintForce = FVector::ZeroVector;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fastener|State")
	int32 ConstraintIndex = -1;
};

UCLASS()
class HSRTRACKDYNAMICS_API AFastenerSpringGrid : public AActor
{
	GENERATED_BODY()

public:
	AFastenerSpringGrid();

	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Params")
	FFastenerStiffnessParams StiffnessParams;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Params")
	float RailMassPerMeter = 60.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Params")
	float RailInertiaZ = 3.0e-5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Params")
	bool bUseUEPhysicsConstraints = true;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Params", meta = (ClampMin = "1", ClampMax = "50000"))
	int32 MaxConstraintBatchSize = 5000;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Params")
	bool bEnableNonlinearStiffness = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Fastener|Params", meta = (EditCondition = "bEnableNonlinearStiffness"))
	float NonlinearStiffnessCoefficient = 0.1f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fastener|State")
	TArray<FFastenerNodeState> FastenerStates;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fastener|Components")
	TArray<UPhysicsConstraintComponent*> PhysicsConstraints;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fastener|Components")
	TArray<UStaticMeshComponent*> RailSegmentMeshes;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Fastener|Components")
	TArray<UStaticMeshComponent*> SleeperBodyMeshes;

	UFUNCTION(BlueprintCallable, Category = "Fastener|Setup")
	void InitializeFastenerGrid(class ATrackGenerator* TrackGen);

	UFUNCTION(BlueprintCallable, Category = "Fastener|Physics")
	void ApplyWheelLoadToRail(int32 FastenerIndex, float VerticalForce);

	UFUNCTION(BlueprintCallable, Category = "Fastener|Physics")
	FVector GetFastenerReactionForce(int32 FastenerIndex) const;

	UFUNCTION(BlueprintCallable, Category = "Fastener|Physics")
	void UpdateCustomSpringForces(float DeltaTime);

	UFUNCTION(BlueprintCallable, Category = "Fastener|Query")
	int32 FindNearestFastenerIndex(const FVector& WorldPosition) const;

	UFUNCTION(BlueprintCallable, Category = "Fastener|Query")
	FVector GetRailDisplacementAtPosition(const FVector& WorldPosition) const;

private:
	void CreatePhysicsConstraint(int32 Index, UStaticMeshComponent* RailSeg, UStaticMeshComponent* SleeperBody, const FTransform& FastenerWorldTransform);
	void ComputeNonlinearCorrection(FFastenerNodeState& NodeState);
	void InterpolateRailDisplacement(const FVector& WorldPosition, FVector& OutDisplacement) const;

	UPROPERTY()
	TMap<int32, int32> PositionToFastenerIndex;

	float CachedSpacing = 0.6f;
};
