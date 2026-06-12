#include "WheelRailContact.h"
#include "HSRTrackDynamics.h"

UWheelRailContact::UWheelRailContact()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UWheelRailContact::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	ComputeCreepage();

	float EffectiveNormalLoad = WheelLoad;

	if (bEnableSpinCreep && FMath::Abs(WheelLateralDisplacement) > KINDA_SMALL_NUMBER)
	{
		float ConeAngle = 0.05f;
		float AdditionalLoad = 2.0f * WheelLoad * ConeAngle * WheelLateralDisplacement / HertzParams.WheelRadius;
		EffectiveNormalLoad += AdditionalLoad;
	}

	EffectiveNormalLoad = FMath::Max(EffectiveNormalLoad, 0.0f);

	LastContactResult = ComputeHertzianContact(EffectiveNormalLoad);

	if (LastContactResult.bInContact)
	{
		FVector LinearCreepForces = ComputeKalkerLinearCreepForces(EffectiveNormalLoad, LastContactResult);
		FVector SaturatedForces = ComputeSaturatedCreepForces(LinearCreepForces, EffectiveNormalLoad);
		LastContactResult.TangentialForce = SaturatedForces;
		LastContactResult.LongitudinalCreepForce = FVector(SaturatedForces.X, 0.0f, 0.0f);
		LastContactResult.LateralCreepForce = FVector(0.0f, SaturatedForces.Y, 0.0f);

		if (bUseSimplifiedFASTSIM)
		{
			FVector FastsimForces = ComputeFASTSIMCreepForces(EffectiveNormalLoad, LastContactResult);
			LastContactResult.TangentialForce = FastsimForces;
		}

		if (bEnableSpinCreep)
		{
			float SpinMoment = KalkerCoeffs.C33 * CurrentCreepage.SpinCreepage *
				ComputeCombinedCurvature() * EffectiveNormalLoad * LastContactResult.ContactPatchSemiAxisA;
			LastContactResult.SpinCreepMoment = SpinMoment;
		}
	}
}

float UWheelRailContact::ComputeCombinedCurvature() const
{
	float A = HertzParams.WheelProfileCurvature;
	float B = -HertzParams.LateralRailCurvature;
	float C = 1.0f / HertzParams.WheelRadius;
	return A + B + C;
}

float UWheelRailContact::ComputeEquivalentElasticModulus() const
{
	float E1 = HertzParams.WheelElasticModulus;
	float E2 = HertzParams.RailElasticModulus;
	float Nu1 = HertzParams.WheelPoissonRatio;
	float Nu2 = HertzParams.RailPoissonRatio;

	float InvEStar = ((1.0f - Nu1 * Nu1) / E1) + ((1.0f - Nu2 * Nu2) / E2);
	return 1.0f / InvEStar;
}

FContactPatchResult UWheelRailContact::ComputeHertzianContact(float NormalLoad)
{
	FContactPatchResult Result;
	Result.NormalForce = NormalLoad;
	Result.bInContact = NormalLoad > 0.0f;

	if (NormalLoad <= 0.0f)
	{
		return Result;
	}

	float A = HertzParams.WheelProfileCurvature;
	float B = HertzParams.LateralRailCurvature;
	float R = 1.0f / HertzParams.WheelProfileCurvature;

	float SumAB = A + B;
	if (SumAB < KINDA_SMALL_NUMBER) SumAB = KINDA_SMALL_NUMBER;

	float DiffAB = FMath::Abs(A - B);
	float CosTheta = DiffAB / SumAB;
	CosTheta = FMath::Clamp(CosTheta, 0.0f, 1.0f);
	float Theta = FMath::Acos(CosTheta);

	float EStar = ComputeEquivalentElasticModulus();

	float Eta, Mu, Lambda;
	{
		float SinT = FMath::Sin(Theta);
		float CosT = FMath::Cos(Theta);
		float KApprox = 1.0f + (PI / 2.0f - 1.0f) * SinT;
		float EApprox = 1.0f + (1.0f - PI / 4.0f) * SinT;

		Eta = KApprox - EApprox;
		Mu = EApprox;
		Lambda = 2.0f * KApprox * EApprox / PI;
	}

	float CombinedR = ComputeCombinedCurvature();
	if (CombinedR < KINDA_SMALL_NUMBER) CombinedR = KINDA_SMALL_NUMBER;

	float C_NS = (3.0f * NormalLoad * Lambda) / (4.0f * EStar * CombinedR);
	float SemiAxisA = FMath::Pow(C_NS, 1.0f / 3.0f) * FMath::Pow(Eta, 1.0f / 3.0f);
	float SemiAxisB = FMath::Pow(C_NS, 1.0f / 3.0f) / FMath::Pow(Eta, 1.0f / 3.0f);

	if (A < B)
	{
		float Temp = SemiAxisA;
		SemiAxisA = SemiAxisB;
		SemiAxisB = Temp;
	}

	Result.ContactPatchSemiAxisA = SemiAxisA;
	Result.ContactPatchSemiAxisB = SemiAxisB;

	float PatchArea = PI * SemiAxisA * SemiAxisB;
	Result.MaxContactPressure = (3.0f * NormalLoad) / (2.0f * PI * SemiAxisA * SemiAxisB);

	float Penetration = FMath::Pow(9.0f * NormalLoad * NormalLoad / (16.0f * EStar * EStar * CombinedR), 1.0f / 3.0f);
	Result.PenetrationDepth = Penetration;

	FVector ContactOffset(0.0f, WheelLateralDisplacement, -(HertzParams.WheelRadius - Penetration));
	Result.ContactPointWorld = WheelCenterPosition + ContactOffset;
	Result.ContactPointWorld.Z = RailTopPosition.Z + RailIrregularityVertical;

	return Result;
}

void UWheelRailContact::ComputeCreepage()
{
	float EffectiveRadius = ComputeEffectiveWheelRadius(WheelLateralDisplacement);
	float V = VehicleSpeed;

	if (FMath::Abs(V) < 0.1f)
	{
		CurrentCreepage.LongitudinalCreepage = 0.0f;
		CurrentCreepage.LateralCreepage = 0.0f;
		CurrentCreepage.SpinCreepage = 0.0f;
		return;
	}

	float VcX = WheelVelocity.X;
	float VcY = WheelVelocity.Y;
	float OmegaR = WheelAngularVelocity * EffectiveRadius;

	CurrentCreepage.LongitudinalCreepage = (VcX - OmegaR) / FMath::Abs(V);
	CurrentCreepage.LateralCreepage = VcY / FMath::Abs(V);

	if (bEnableSpinCreep)
	{
		float ConeAngle = 0.05f;
		CurrentCreepage.SpinCreepage = ConeAngle / EffectiveRadius;
	}

	ApplyRailIrregularityToCreepage();
}

void UWheelRailContact::ApplyRailIrregularityToCreepage()
{
	float IrregularitySlopeV = RailIrregularityVertical * 10.0f;
	float IrregularitySlopeL = RailIrregularityLateral * 10.0f;

	float V = FMath::Abs(VehicleSpeed);
	if (V > 0.1f)
	{
		CurrentCreepage.LongitudinalCreepage += IrregularitySlopeV / V;
		CurrentCreepage.LateralCreepage += IrregularitySlopeL / V;
	}
}

FVector UWheelRailContact::ComputeKalkerLinearCreepForces(float NormalLoad, const FContactPatchResult& Patch)
{
	float A = Patch.ContactPatchSemiAxisA;
	float B = Patch.ContactPatchSemiAxisB;

	if (A < KINDA_SMALL_NUMBER || B < KINDA_SMALL_NUMBER) return FVector::ZeroVector;

	float EStar = ComputeEquivalentElasticModulus();
	float AB = A * B;
	float C = 2.0f * AB * EStar;

	float Fx = -C * KalkerCoeffs.C11 * CurrentCreepage.LongitudinalCreepage;
	float Fy = -C * KalkerCoeffs.C22 * CurrentCreepage.LateralCreepage;

	if (bEnableSpinCreep)
	{
		Fy += -C * KalkerCoeffs.C23 * CurrentCreepage.SpinCreepage * A;
	}

	return FVector(Fx, Fy, 0.0f);
}

FVector UWheelRailContact::ComputeSaturatedCreepForces(const FVector& LinearForce, float NormalLoad)
{
	float LinMag = LinearForce.Size();
	float MaxFriction = StaticFrictionCoefficient * NormalLoad;

	if (LinMag <= MaxFriction)
	{
		return LinearForce;
	}

	float KineticFriction = KineticFrictionCoefficient * NormalLoad;

	float TransitionSpeed = CreepageSaturationThreshold;
	float CreepMag = FMath::Sqrt(
		CurrentCreepage.LongitudinalCreepage * CurrentCreepage.LongitudinalCreepage +
		CurrentCreepage.LateralCreepage * CurrentCreepage.LateralCreepage
	);

	float SaturatedForceMag;
	if (CreepMag < TransitionSpeed)
	{
		float Ratio = CreepMag / TransitionSpeed;
		SaturatedForceMag = FMath::Lerp(LinMag, MaxFriction, Ratio);
	}
	else
	{
		float ExcessRatio = (CreepMag - TransitionSpeed) / TransitionSpeed;
		SaturatedForceMag = FMath::Lerp(MaxFriction, KineticFriction, FMath::Min(ExcessRatio, 1.0f));
	}

	if (LinMag > KINDA_SMALL_NUMBER)
	{
		return LinearForce.GetUnsafeNormal() * SaturatedForceMag;
	}

	return FVector::ZeroVector;
}

FVector UWheelRailContact::ComputeFASTSIMCreepForces(float NormalLoad, const FContactPatchResult& Patch)
{
	float A = Patch.ContactPatchSemiAxisA;
	float B = Patch.ContactPatchSemiAxisB;

	if (A < KINDA_SMALL_NUMBER || B < KINDA_SMALL_NUMBER) return FVector::ZeroVector;

	float EStar = ComputeEquivalentElasticModulus();
	float L1 = 8.0f * A / (3.0f * EStar * KalkerCoeffs.C11 * PI);
	float L2 = 8.0f * A / (3.0f * EStar * KalkerCoeffs.C22 * PI);
	float L3 = 8.0f * A / (3.0f * EStar * KalkerCoeffs.C23 * PI);

	int32 NStrips = 5;
	float StripWidth = 2.0f * B / NStrips;

	float Fx = 0.0f, Fy = 0.0f;

	for (int32 i = 0; i < NStrips; ++i)
	{
		float Y = -B + (i + 0.5f) * StripWidth;
		float LocalA = A * FMath::Sqrt(1.0f - (Y * Y) / (B * B));

		float P0 = (3.0f * NormalLoad) / (2.0f * PI * A * B);
		float Py = P0 * FMath::Sqrt(FMath::Max(0.0f, 1.0f - (Y * Y) / (B * B)));

		int32 NSteps = 4;
		float dx = 2.0f * LocalA / NSteps;

		float Sx = 0.0f, Sy = 0.0f;

		for (int32 j = 0; j < NSteps; ++j)
		{
			float X = -LocalA + (j + 0.5f) * dx;

			float ElasticLimit = StaticFrictionCoefficient * Py * dx;

			float dSx = (CurrentCreepage.LongitudinalCreepage * dx / L1);
			float dSy = (CurrentCreepage.LateralCreepage * dx / L2 +
				CurrentCreepage.SpinCreepage * Y * dx / L3);

			Sx += dSx;
			Sy += dSy;

			float SMag = FMath::Sqrt(Sx * Sx + Sy * Sy);

			if (SMag > ElasticLimit && SMag > KINDA_SMALL_NUMBER)
			{
				float Scale = ElasticLimit / SMag;
				Sx *= Scale;
				Sy *= Scale;
			}

			Fx += Sx * StripWidth;
			Fy += Sy * StripWidth;
		}
	}

	Fx /= L1;
	Fy /= L2;

	return FVector(-Fx, -Fy, 0.0f);
}

float UWheelRailContact::ComputeEffectiveWheelRadius(float LateralShift) const
{
	float ConeAngle = 0.05f;
	return HertzParams.WheelRadius - ConeAngle * FMath::Abs(LateralShift);
}

float UWheelRailContact::GetRollingResistanceForce() const
{
	float RollingResistCoeff = 0.001f;
	return RollingResistCoeff * WheelLoad;
}
