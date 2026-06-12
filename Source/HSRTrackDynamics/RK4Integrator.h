#pragma once

#include "CoreMinimal.h"
#include "HSRTrackDynamics.h"

template <typename StateType, typename DerivType>
class TRK4Integrator
{
public:
	typedef TFunction<DerivType(const StateType& State, float Time)> FDerivativeFunc;

	static StateType Integrate(
		const StateType& InitialState,
		float Time,
		float Dt,
		const FDerivativeFunc& DerivativeFunc)
	{
		DerivType K1 = DerivativeFunc(InitialState, Time);
		DerivType K2 = DerivativeFunc(StateFromDerivative(InitialState, K1, Dt * 0.5f), Time + Dt * 0.5f);
		DerivType K3 = DerivativeFunc(StateFromDerivative(InitialState, K2, Dt * 0.5f), Time + Dt * 0.5f);
		DerivType K4 = DerivativeFunc(StateFromDerivative(InitialState, K3, Dt), Time + Dt);

		StateType Result = InitialState;
		AccumulateRK4(Result, K1, K2, K3, K4, Dt);
		return Result;
	}

	static StateType IntegrateWithAdaptiveStep(
		const StateType& InitialState,
		float Time,
		float& InOutDt,
		float Tolerance,
		float MinDt,
		float MaxDt,
		const FDerivativeFunc& DerivativeFunc,
		int32& OutSubsteps)
	{
		OutSubsteps = 0;
		float CurrentTime = Time;
		StateType CurrentState = InitialState;
		float CurrentDt = InOutDt;

		while (CurrentDt > KINDA_SMALL_NUMBER)
		{
			StateType FullStep = Integrate(CurrentState, CurrentTime, CurrentDt, DerivativeFunc);

			float HalfDt = CurrentDt * 0.5f;
			StateType HalfStep1 = Integrate(CurrentState, CurrentTime, HalfDt, DerivativeFunc);
			StateType HalfStep2 = Integrate(HalfStep1, CurrentTime + HalfDt, HalfDt, DerivativeFunc);

			float Error = ComputeStateError(FullStep, HalfStep2);

			if (Error <= Tolerance || CurrentDt <= MinDt)
			{
				CurrentState = HalfStep2;
				CurrentTime += CurrentDt;
				OutSubsteps++;

				if (Error < Tolerance * 0.5f && CurrentDt < MaxDt)
				{
					CurrentDt = FMath::Min(CurrentDt * 1.5f, MaxDt);
				}
			}
			else
			{
				CurrentDt = FMath::Max(CurrentDt * 0.5f, MinDt);
			}

			if (OutSubsteps > 1000) break;
		}

		InOutDt = CurrentDt;
		return CurrentState;
	}

private:
	static StateType StateFromDerivative(const StateType& Base, const DerivType& Deriv, float Scale)
	{
		StateType Result = Base;
		AddDerivativeToState(Result, Deriv, Scale);
		return Result;
	}

	static void AddDerivativeToState(StateType& State, const DerivType& Deriv, float Scale)
	{
		static_assert(sizeof(StateType) == 0, "Must specialize AddDerivativeToState for your state type");
	}

	static void AccumulateRK4(StateType& State, const DerivType& K1, const DerivType& K2, const DerivType& K3, const DerivType& K4, float Dt)
	{
		static_assert(sizeof(StateType) == 0, "Must specialize AccumulateRK4 for your state type");
	}

	static float ComputeStateError(const StateType& A, const StateType& B)
	{
		static_assert(sizeof(StateType) == 0, "Must specialize ComputeStateError for your state type");
		return 0.0f;
	}
};

USTRUCT(BlueprintType)
struct FRK4WheelsetState
{
	GENERATED_BODY()

	UPROPERTY()
	FVector Position = FVector::ZeroVector;

	UPROPERTY()
	FVector Velocity = FVector::ZeroVector;

	UPROPERTY()
	FQuat Rotation = FQuat::Identity;

	UPROPERTY()
	FVector AngularVelocity = FVector::ZeroVector;

	UPROPERTY()
	float WheelSpinAngle = 0.0f;

	UPROPERTY()
	float WheelSpinVelocity = 0.0f;
};

USTRUCT()
struct FRK4WheelsetDerivative
{
	GENERATED_BODY()

	FVector LinearVelocity = FVector::ZeroVector;
	FVector LinearAcceleration = FVector::ZeroVector;
	FVector AngularVelocity = FVector::ZeroVector;
	FVector AngularAcceleration = FVector::ZeroVector;
	float SpinVelocity = 0.0f;
	float SpinAcceleration = 0.0f;
};

template <>
FORCEINLINE void TRK4Integrator<FRK4WheelsetState, FRK4WheelsetDerivative>::AddDerivativeToState(
	FRK4WheelsetState& State, const FRK4WheelsetDerivative& Deriv, float Scale)
{
	State.Position += Deriv.LinearVelocity * Scale;
	State.Velocity += Deriv.LinearAcceleration * Scale;

	FQuat DeltaQuat = FQuat(Deriv.AngularVelocity * Scale * 0.5f, 0.0f) * State.Rotation;
	State.Rotation = (State.Rotation + DeltaQuat).GetNormalized();

	State.AngularVelocity += Deriv.AngularAcceleration * Scale;
	State.WheelSpinAngle += Deriv.SpinVelocity * Scale;
	State.WheelSpinVelocity += Deriv.SpinAcceleration * Scale;
}

template <>
FORCEINLINE void TRK4Integrator<FRK4WheelsetState, FRK4WheelsetDerivative>::AccumulateRK4(
	FRK4WheelsetState& State,
	const FRK4WheelsetDerivative& K1, const FRK4WheelsetDerivative& K2,
	const FRK4WheelsetDerivative& K3, const FRK4WheelsetDerivative& K4, float Dt)
{
	float SixthDt = Dt / 6.0f;

	State.Position += (K1.LinearVelocity + 2.0f * K2.LinearVelocity + 2.0f * K3.LinearVelocity + K4.LinearVelocity) * SixthDt;
	State.Velocity += (K1.LinearAcceleration + 2.0f * K2.LinearAcceleration + 2.0f * K3.LinearAcceleration + K4.LinearAcceleration) * SixthDt;

	FVector AvgAngVel = (K1.AngularVelocity + 2.0f * K2.AngularVelocity + 2.0f * K3.AngularVelocity + K4.AngularVelocity) * SixthDt;
	FQuat DeltaQuat = FQuat(AvgAngVel * 0.5f, 0.0f) * State.Rotation;
	State.Rotation = (State.Rotation + DeltaQuat).GetNormalized();

	State.AngularVelocity += (K1.AngularAcceleration + 2.0f * K2.AngularAcceleration + 2.0f * K3.AngularAcceleration + K4.AngularAcceleration) * SixthDt;
	State.WheelSpinAngle += (K1.SpinVelocity + 2.0f * K2.SpinVelocity + 2.0f * K3.SpinVelocity + K4.SpinVelocity) * SixthDt;
	State.WheelSpinVelocity += (K1.SpinAcceleration + 2.0f * K2.SpinAcceleration + 2.0f * K3.SpinAcceleration + K4.SpinAcceleration) * SixthDt;
}

template <>
FORCEINLINE float TRK4Integrator<FRK4WheelsetState, FRK4WheelsetDerivative>::ComputeStateError(
	const FRK4WheelsetState& A, const FRK4WheelsetState& B)
{
	float PosError = FVector::Dist(A.Position, B.Position);
	float VelError = FVector::Dist(A.Velocity, B.Velocity);
	float AngVelError = FVector::Dist(A.AngularVelocity, B.AngularVelocity);
	return PosError * 1000.0f + VelError * 100.0f + AngVelError * 10.0f;
}

USTRUCT(BlueprintType)
struct FRK4CarBodyState
{
	GENERATED_BODY()

	UPROPERTY()
	FVector Position = FVector::ZeroVector;

	UPROPERTY()
	FVector Velocity = FVector::ZeroVector;

	UPROPERTY()
	FQuat Rotation = FQuat::Identity;

	UPROPERTY()
	FVector AngularVelocity = FVector::ZeroVector;
};

USTRUCT()
struct FRK4CarBodyDerivative
{
	GENERATED_BODY()

	FVector LinearVelocity = FVector::ZeroVector;
	FVector LinearAcceleration = FVector::ZeroVector;
	FVector AngularVelocity = FVector::ZeroVector;
	FVector AngularAcceleration = FVector::ZeroVector;
};

template <>
FORCEINLINE void TRK4Integrator<FRK4CarBodyState, FRK4CarBodyDerivative>::AddDerivativeToState(
	FRK4CarBodyState& State, const FRK4CarBodyDerivative& Deriv, float Scale)
{
	State.Position += Deriv.LinearVelocity * Scale;
	State.Velocity += Deriv.LinearAcceleration * Scale;
	FQuat DeltaQuat = FQuat(Deriv.AngularVelocity * Scale * 0.5f, 0.0f) * State.Rotation;
	State.Rotation = (State.Rotation + DeltaQuat).GetNormalized();
	State.AngularVelocity += Deriv.AngularAcceleration * Scale;
}

template <>
FORCEINLINE void TRK4Integrator<FRK4CarBodyState, FRK4CarBodyDerivative>::AccumulateRK4(
	FRK4CarBodyState& State,
	const FRK4CarBodyDerivative& K1, const FRK4CarBodyDerivative& K2,
	const FRK4CarBodyDerivative& K3, const FRK4CarBodyDerivative& K4, float Dt)
{
	float SixthDt = Dt / 6.0f;
	State.Position += (K1.LinearVelocity + 2.0f * K2.LinearVelocity + 2.0f * K3.LinearVelocity + K4.LinearVelocity) * SixthDt;
	State.Velocity += (K1.LinearAcceleration + 2.0f * K2.LinearAcceleration + 2.0f * K3.LinearAcceleration + K4.LinearAcceleration) * SixthDt;
	FVector AvgAngVel = (K1.AngularVelocity + 2.0f * K2.AngularVelocity + 2.0f * K3.AngularVelocity + K4.AngularVelocity) * SixthDt;
	FQuat DeltaQuat = FQuat(AvgAngVel * 0.5f, 0.0f) * State.Rotation;
	State.Rotation = (State.Rotation + DeltaQuat).GetNormalized();
	State.AngularVelocity += (K1.AngularAcceleration + 2.0f * K2.AngularAcceleration + 2.0f * K3.AngularAcceleration + K4.AngularAcceleration) * SixthDt;
}

template <>
FORCEINLINE float TRK4Integrator<FRK4CarBodyState, FRK4CarBodyDerivative>::ComputeStateError(
	const FRK4CarBodyState& A, const FRK4CarBodyState& B)
{
	return FVector::Dist(A.Position, B.Position) * 1000.0f
		+ FVector::Dist(A.Velocity, B.Velocity) * 100.0f
		+ FVector::Dist(A.AngularVelocity, B.AngularVelocity) * 10.0f;
}
