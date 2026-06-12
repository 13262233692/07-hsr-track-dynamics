#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ProceduralMeshComponent.h"
#include "TrackGenerator.generated.h"

USTRUCT(BlueprintType)
struct FTrackProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail|Profile")
	float RailHeight = 0.176f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail|Profile")
	float RailFootWidth = 0.150f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail|Profile")
	float RailHeadWidth = 0.070f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail|Profile")
	float RailWebThickness = 0.016f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rail|Profile")
	float RailHeadRadius = 0.030f;
};

USTRUCT(BlueprintType)
struct FSleeperProfile
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sleeper|Profile")
	float SleeperLength = 2.6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sleeper|Profile")
	float SleeperWidth = 0.25f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sleeper|Profile")
	float SleeperHeight = 0.20f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Sleeper|Profile")
	float SleeperSpacing = 0.60f;
};

UCLASS()
class HSRTRACKDYNAMICS_API ATrackGenerator : public AActor
{
	GENERATED_BODY()

public:
	ATrackGenerator();

	virtual void OnConstruction(const FTransform& Transform) override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Layout")
	float TrackLength = 5000.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Layout")
	float Gauge = 1.435f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Layout")
	float CurvatureRadius = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Layout")
	float CantAngle = 0.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Layout")
	int32 SegmentLength = 10;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Profile")
	FTrackProfile RailProfile;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Profile")
	FSleeperProfile SleeperProfile;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Ballast")
	float BallastTopWidth = 3.6f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Ballast")
	float BallastBottomWidth = 4.5f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Track|Ballast")
	float BallastHeight = 0.35f;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track|Components")
	UProceduralMeshComponent* LeftRailMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track|Components")
	UProceduralMeshComponent* RightRailMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track|Components")
	UProceduralMeshComponent* SleeperMesh;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Track|Components")
	UProceduralMeshComponent* BallastMesh;

	UFUNCTION(BlueprintCallable, Category = "Track|Generation")
	void GenerateTrack();

	UFUNCTION(BlueprintCallable, Category = "Track|Query")
	FVector GetRailWorldPosition(float DistanceAlongTrack, bool bLeftRail) const;

	UFUNCTION(BlueprintCallable, Category = "Track|Query")
	FTransform GetRailWorldTransform(float DistanceAlongTrack, bool bLeftRail) const;

	UFUNCTION(BlueprintCallable, Category = "Track|Query")
	int32 GetFastenerCount() const;

	UFUNCTION(BlueprintCallable, Category = "Track|Query")
	FVector GetFastenerWorldPosition(int32 FastenerIndex) const;

	UFUNCTION(BlueprintCallable, Category = "Track|Query")
	float GetTotalFastenerSpacing() const;

private:
	void GenerateRailMesh(bool bLeftRail);
	void GenerateSleeperMesh();
	void GenerateBallastMesh();
	void BuildCrossSectionProfile(bool bLeftRail, TArray<FVector>& OutVertices, TArray<int32>& OutTriangles);
	void ComputeTrackCurvePoint(float Distance, FVector& OutPosition, FVector& OutTangent, FVector& OutNormal, FVector& OutBinormal) const;

	TArray<FVector> CachedLeftRailPositions;
	TArray<FVector> CachedRightRailPositions;
	TArray<FVector> CachedFastenerPositions;
	TArray<FVector> CachedTangentVectors;
	TArray<FVector> CachedNormalVectors;

	static constexpr int32 RailCrossSectionSides = 12;
};
