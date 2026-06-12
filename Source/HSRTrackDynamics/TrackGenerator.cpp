#include "TrackGenerator.h"
#include "HSRTrackDynamics.h"

ATrackGenerator::ATrackGenerator()
{
	PrimaryActorTick.bCanEverTick = false;

	LeftRailMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("LeftRailMesh"));
	LeftRailMesh->SetupAttachment(RootComponent);

	RightRailMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("RightRailMesh"));
	RightRailMesh->SetupAttachment(RootComponent);

	SleeperMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("SleeperMesh"));
	SleeperMesh->SetupAttachment(RootComponent);

	BallastMesh = CreateDefaultSubobject<UProceduralMeshComponent>(TEXT("BallastMesh"));
	BallastMesh->SetupAttachment(RootComponent);
}

void ATrackGenerator::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	GenerateTrack();
}

void ATrackGenerator::GenerateTrack()
{
	CachedLeftRailPositions.Empty();
	CachedRightRailPositions.Empty();
	CachedFastenerPositions.Empty();
	CachedTangentVectors.Empty();
	CachedNormalVectors.Empty();

	GenerateRailMesh(true);
	GenerateRailMesh(false);
	GenerateSleeperMesh();
	GenerateBallastMesh();

	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Track generated: %.1f km, %d fasteners, gauge %.3f m"),
		TrackLength / 1000.0f, GetFastenerCount(), Gauge);
}

void ATrackGenerator::ComputeTrackCurvePoint(float Distance, FVector& OutPosition, FVector& OutTangent, FVector& OutNormal, FVector& OutBinormal) const
{
	float S = FMath::Clamp(Distance, 0.0f, TrackLength);

	if (CurvatureRadius > KINDA_SMALL_NUMBER)
	{
		float Angle = S / CurvatureRadius;
		OutPosition = FVector(S * FMath::Sin(Angle) / Angle, CurvatureRadius * (1.0f - FMath::Cos(Angle)), 0.0f);
		OutTangent = FVector(FMath::Cos(Angle), FMath::Sin(Angle), 0.0f);
	}
	else
	{
		OutPosition = FVector(S, 0.0f, 0.0f);
		OutTangent = FVector(1.0f, 0.0f, 0.0f);
	}

	OutNormal = FVector(0.0f, 0.0f, 1.0f);

	float CantRad = FMath::DegreesToRadians(CantAngle);
	FVector LateralDir = FVector::CrossProduct(OutTangent, OutNormal).GetSafeNormal();
	OutNormal = FMath::Cos(CantRad) * OutNormal + FMath::Sin(CantRad) * LateralDir;
	OutBinormal = FVector::CrossProduct(OutTangent, OutNormal).GetSafeNormal();

	OutPosition = GetActorTransform().TransformPosition(OutPosition);
	OutTangent = GetActorTransform().TransformVector(OutTangent);
	OutNormal = GetActorTransform().TransformVector(OutNormal);
	OutBinormal = GetActorTransform().TransformVector(OutBinormal);
}

void ATrackGenerator::GenerateRailMesh(bool bLeftRail)
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FLinearColor> Colors;
	TArray<FProcMeshTangent> Tangents;

	float LateralOffset = bLeftRail ? -Gauge / 2.0f : Gauge / 2.0f;
	int32 NumSegments = FMath::CeilToInt(TrackLength / (float)SegmentLength);

	TArray<FVector> SectionPoints;
	SectionPoints.Reserve(NumSegments + 1);

	for (int32 i = 0; i <= NumSegments; ++i)
	{
		float Dist = FMath::Min((float)i * SegmentLength, TrackLength);
		FVector Pos, Tan, Norm, Binorm;
		ComputeTrackCurvePoint(Dist, Pos, Tan, Norm, Binorm);
		FVector RailPos = Pos + LateralOffset * Binorm + RailProfile.RailHeight * Norm;
		SectionPoints.Add(RailPos);

		if (!bLeftRail)
		{
			CachedRightRailPositions.Add(RailPos);
		}
		else
		{
			CachedLeftRailPositions.Add(RailPos);
		}
	}

	float ProfileScale = 1.0f;
	int32 NumProfileVerts = RailCrossSectionSides * 2;

	auto AddRailProfile = [&](const FVector& Center, const FVector& InTangent, const FVector& InNormal, const FVector& InBinormal, float V)
	{
		float HalfFoot = RailProfile.RailFootWidth / 2.0f * ProfileScale;
		float HalfHead = RailProfile.RailHeadWidth / 2.0f * ProfileScale;
		float H = RailProfile.RailHeight * ProfileScale;
		float WebHalf = RailProfile.RailWebThickness / 2.0f * ProfileScale;

		TArray<FVector2D> Profile;
		Profile.Reserve(RailCrossSectionSides);

		int32 FootPoints = 3;
		for (int32 j = 0; j < FootPoints; ++j)
		{
			float Alpha = (float)j / (FootPoints - 1);
			float X = FMath::Lerp(-HalfFoot, HalfFoot, Alpha);
			Profile.Add(FVector2D(X, 0.0f));
		}

		int32 WebFootPoints = 2;
		for (int32 j = 0; j < WebFootPoints; ++j)
		{
			float Alpha = (float)j / (WebFootPoints - 1);
			float X = FMath::Lerp(-WebHalf, WebHalf, Alpha);
			Profile.Add(FVector2D(X, H * 0.35f));
		}

		int32 WebHeadPoints = 2;
		for (int32 j = 0; j < WebHeadPoints; ++j)
		{
			float Alpha = (float)j / (WebHeadPoints - 1);
			float X = FMath::Lerp(-WebHalf, WebHalf, Alpha);
			Profile.Add(FVector2D(X, H * 0.65f));
		}

		int32 HeadPoints = 3;
		for (int32 j = 0; j < HeadPoints; ++j)
		{
			float Alpha = (float)j / (HeadPoints - 1);
			float X = FMath::Lerp(-HalfHead, HalfHead, Alpha);
			Profile.Add(FVector2D(X, H));
		}

		for (const auto& Pt : Profile)
		{
			FVector WorldPos = Center + Pt.X * InBinormal + Pt.Y * InNormal;
			Vertices.Add(WorldPos);
			Normals.Add(InNormal);
			UVs.Add(FVector2D(Pt.X / HalfFoot + 0.5f, V));
			Colors.Add(FLinearColor::White);
			Tangents.Add(FProcMeshTangent(InTangent, false));
		}
	};

	for (int32 i = 0; i <= NumSegments; ++i)
	{
		float Dist = FMath::Min((float)i * SegmentLength, TrackLength);
		FVector Pos, Tan, Norm, Binorm;
		ComputeTrackCurvePoint(Dist, Pos, Tan, Norm, Binorm);
		FVector RailCenter = Pos + LateralOffset * Binorm + RailProfile.RailHeight * Norm;
		float V = Dist / TrackLength;
		AddRailProfile(RailCenter, Tan, Norm, Binorm, V);
	}

	int32 ProfileVertCount = 10 + 2 + 2 + 3;

	for (int32 i = 0; i < NumSegments; ++i)
	{
		for (int32 j = 0; j < ProfileVertCount - 1; ++j)
		{
			int32 Bl = i * ProfileVertCount + j;
			int32 Br = Bl + 1;
			int32 Tl = (i + 1) * ProfileVertCount + j;
			int32 Tr = Tl + 1;

			Triangles.Add(Bl);
			Triangles.Add(Tl);
			Triangles.Add(Br);

			Triangles.Add(Br);
			Triangles.Add(Tl);
			Triangles.Add(Tr);
		}
	}

	UProceduralMeshComponent* MeshComp = bLeftRail ? LeftRailMesh : RightRailMesh;
	MeshComp->CreateMeshSection_LinearColor(
		bLeftRail ? 0 : 1,
		Vertices, Triangles, Normals, UVs, Colors, Tangents, true
	);

	MeshComp->bUseComplexAsSimpleCollision = false;

	FVector BoxExtent(
		TrackLength / 2.0f,
		RailProfile.RailFootWidth / 2.0f,
		RailProfile.RailHeight / 2.0f
	);
}

void ATrackGenerator::GenerateSleeperMesh()
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FLinearColor> Colors;
	TArray<FProcMeshTangent> Tangents;

	int32 NumSleepers = FMath::FloorToInt(TrackLength / SleeperProfile.SleeperSpacing);
	float HalfL = SleeperProfile.SleeperLength / 2.0f;
	float HalfW = SleeperProfile.SleeperWidth / 2.0f;
	float HalfH = SleeperProfile.SleeperHeight / 2.0f;

	for (int32 i = 0; i < NumSleepers; ++i)
	{
		float Dist = i * SleeperProfile.SleeperSpacing + SleeperProfile.SleeperSpacing / 2.0f;
		if (Dist > TrackLength) break;

		FVector Pos, Tan, Norm, Binorm;
		ComputeTrackCurvePoint(Dist, Pos, Tan, Norm, Binorm);

		FVector SleeperCenter = Pos - HalfH * Norm;

		CachedFastenerPositions.Add(Pos - (Gauge / 2.0f) * Binorm);
		CachedFastenerPositions.Add(Pos + (Gauge / 2.0f) * Binorm);

		int32 BaseIdx = Vertices.Num();

		auto AddBoxFace = [&](const TArray<FVector>& FaceVerts, const FVector& FaceNormal, const FVector2D& UVOffset)
		{
			int32 Idx = Vertices.Num();
			for (const auto& V : FaceVerts)
			{
				Vertices.Add(V);
				Normals.Add(FaceNormal);
				UVs.Add(FVector2D(0.0f, 0.0f));
				Colors.Add(FLinearColor(0.4f, 0.35f, 0.25f));
				Tangents.Add(FProcMeshTangent(Tan, false));
			}
			Triangles.Add(Idx);
			Triangles.Add(Idx + 1);
			Triangles.Add(Idx + 2);
			Triangles.Add(Idx);
			Triangles.Add(Idx + 2);
			Triangles.Add(Idx + 3);
		};

		FVector C = SleeperCenter;
		FVector F = HalfW * Tan;
		FVector R = HalfL * Binorm;
		FVector U = HalfH * Norm;

		TArray<FVector> TopFace = { C - F + R + U, C + F + R + U, C + F - R + U, C - F - R + U };
		TArray<FVector> BottomFace = { C - F - R - U, C + F - R - U, C + F + R - U, C - F + R - U };
		TArray<FVector> FrontFace = { C + F - R - U, C + F - R + U, C + F + R + U, C + F + R - U };
		TArray<FVector> BackFace = { C - F + R - U, C - F - R - U, C - F - R + U, C - F + R + U };
		TArray<FVector> LeftFace = { C - F - R - U, C - F - R + U, C + F - R + U, C + F - R - U };
		TArray<FVector> RightFace = { C + F + R - U, C + F + R + U, C - F + R + U, C - F + R - U };

		AddBoxFace(TopFace, Norm, FVector2D(0, 0));
		AddBoxFace(BottomFace, -Norm, FVector2D(0, 1));
		AddBoxFace(FrontFace, Tan, FVector2D(1, 0));
		AddBoxFace(BackFace, -Tan, FVector2D(0, 0));
		AddBoxFace(LeftFace, -Binorm, FVector2D(0, 0));
		AddBoxFace(RightFace, Binorm, FVector2D(1, 0));
	}

	SleeperMesh->CreateMeshSection_LinearColor(
		2, Vertices, Triangles, Normals, UVs, Colors, Tangents, true
	);
}

void ATrackGenerator::GenerateBallastMesh()
{
	TArray<FVector> Vertices;
	TArray<int32> Triangles;
	TArray<FVector> Normals;
	TArray<FVector2D> UVs;
	TArray<FLinearColor> Colors;
	TArray<FProcMeshTangent> Tangents;

	int32 NumSegments = FMath::CeilToInt(TrackLength / (float)SegmentLength);
	float HalfTopW = BallastTopWidth / 2.0f;
	float HalfBotW = BallastBottomWidth / 2.0f;
	float H = BallastHeight;

	auto AddWedgeSection = [&](int32 SegIdx)
	{
		float D0 = SegIdx * SegmentLength;
		float D1 = FMath::Min((SegIdx + 1) * (float)SegmentLength, TrackLength);

		FVector P0, T0, N0, B0;
		ComputeTrackCurvePoint(D0, P0, T0, N0, B0);
		FVector P1, T1, N1, B1;
		ComputeTrackCurvePoint(D1, P1, T1, N1, B1);

		FVector Center0 = P0 - H * N0;
		FVector Center1 = P1 - H * N1;

		int32 BaseIdx = Vertices.Num();

		Vertices.Add(Center0 + HalfTopW * B0);
		Vertices.Add(Center0 - HalfTopW * B0);
		Vertices.Add(Center0 - HalfBotW * B0 - H * N0);
		Vertices.Add(Center0 + HalfBotW * B0 - H * N0);

		Vertices.Add(Center1 + HalfTopW * B1);
		Vertices.Add(Center1 - HalfTopW * B1);
		Vertices.Add(Center1 - HalfBotW * B1 - H * N1);
		Vertices.Add(Center1 + HalfBotW * B1 - H * N1);

		for (int32 v = 0; v < 8; ++v)
		{
			Normals.Add(N0);
			UVs.Add(FVector2D(0, 0));
			Colors.Add(FLinearColor(0.55f, 0.50f, 0.45f));
			Tangents.Add(FProcMeshTangent(T0, false));
		}

		auto AddQuad = [&](int32 A, int32 B, int32 C, int32 D)
		{
			Triangles.Add(BaseIdx + A);
			Triangles.Add(BaseIdx + B);
			Triangles.Add(BaseIdx + C);
			Triangles.Add(BaseIdx + A);
			Triangles.Add(BaseIdx + C);
			Triangles.Add(BaseIdx + D);
		};

		AddQuad(0, 4, 5, 1);
		AddQuad(1, 5, 6, 2);
		AddQuad(2, 6, 7, 3);
		AddQuad(3, 7, 4, 0);
		AddQuad(0, 1, 2, 3);
		AddQuad(4, 7, 6, 5);
	};

	for (int32 i = 0; i < NumSegments; ++i)
	{
		AddWedgeSection(i);
	}

	BallastMesh->CreateMeshSection_LinearColor(
		3, Vertices, Triangles, Normals, UVs, Colors, Tangents, true
	);
}

FVector ATrackGenerator::GetRailWorldPosition(float DistanceAlongTrack, bool bLeftRail) const
{
	FVector Pos, Tan, Norm, Binorm;
	ComputeTrackCurvePoint(DistanceAlongTrack, Pos, Tan, Norm, Binorm);
	float Offset = bLeftRail ? -Gauge / 2.0f : Gauge / 2.0f;
	return Pos + Offset * Binorm + RailProfile.RailHeight * Norm;
}

FTransform ATrackGenerator::GetRailWorldTransform(float DistanceAlongTrack, bool bLeftRail) const
{
	FVector Pos, Tan, Norm, Binorm;
	ComputeTrackCurvePoint(DistanceAlongTrack, Pos, Tan, Norm, Binorm);
	float Offset = bLeftRail ? -Gauge / 2.0f : Gauge / 2.0f;
	FVector RailPos = Pos + Offset * Binorm + RailProfile.RailHeight * Norm;

	FQuat Rotation = FQuat::FindFromVectors(FVector::ForwardVector, Tan);
	return FTransform(Rotation, RailPos);
}

int32 ATrackGenerator::GetFastenerCount() const
{
	return CachedFastenerPositions.Num();
}

FVector ATrackGenerator::GetFastenerWorldPosition(int32 FastenerIndex) const
{
	if (CachedFastenerPositions.IsValidIndex(FastenerIndex))
	{
		return CachedFastenerPositions[FastenerIndex];
	}
	return FVector::ZeroVector;
}

float ATrackGenerator::GetTotalFastenerSpacing() const
{
	return SleeperProfile.SleeperSpacing;
}
