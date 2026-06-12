#include "TrackDynamicsSolver.h"
#include "HSRTrackDynamics.h"
#include "Kismet/GameplayStatics.h"
#include "SubsidenceOperator.h"
#include "DerailmentAssessor.h"

ATrackDynamicsSolver::ATrackDynamicsSolver()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
}

void ATrackDynamicsSolver::BeginPlay()
{
	Super::BeginPlay();
}

void ATrackDynamicsSolver::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!bSimulationRunning) return;

	float EffectiveDt = bUseFixedSubstep ? PhysicsSubstepSize : DeltaTime;
	int32 NumSubsteps = FMath::CeilToInt(DeltaTime / EffectiveDt);
	NumSubsteps = FMath::Min(NumSubsteps, MaxSubstepsPerFrame);

	float SubDt = DeltaTime / NumSubsteps;

	for (int32 i = 0; i < NumSubsteps; ++i)
	{
		ExecutePhysicsSubstep(SubDt);
	}

	UpdateDiagnostics(DeltaTime);
}

void ATrackDynamicsSolver::InitializeSimulation()
{
	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Initializing Track Dynamics Solver..."));

	if (!TrackGeneratorRef)
	{
		UE_LOG(LogHSRTrackDynamics, Warning, TEXT("TrackGenerator not assigned, searching..."));
		TArray<AActor*> FoundActors;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ATrackGenerator::StaticClass(), FoundActors);
		if (FoundActors.Num() > 0)
		{
			TrackGeneratorRef = Cast<ATrackGenerator>(FoundActors[0]);
		}
	}

	if (!FastenerGridRef)
	{
		TArray<AActor*> FoundActors;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), AFastenerSpringGrid::StaticClass(), FoundActors);
		if (FoundActors.Num() > 0)
		{
			FastenerGridRef = Cast<AFastenerSpringGrid>(FoundActors[0]);
		}
	}

	if (!TrainVehicleRef)
	{
		TArray<AActor*> FoundActors;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ATrainMBSVehicle::StaticClass(), FoundActors);
		if (FoundActors.Num() > 0)
		{
			TrainVehicleRef = Cast<ATrainMBSVehicle>(FoundActors[0]);
		}
	}

	if (!SubsidenceOperatorRef)
	{
		TArray<AActor*> FoundActors;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), ASubsidenceOperator::StaticClass(), FoundActors);
		if (FoundActors.Num() > 0)
		{
			SubsidenceOperatorRef = Cast<ASubsidenceOperator>(FoundActors[0]);
		}
	}

	if (SubsidenceOperatorRef)
	{
		if (!SubsidenceOperatorRef->TrackGeneratorRef) SubsidenceOperatorRef->TrackGeneratorRef = TrackGeneratorRef;
		if (!SubsidenceOperatorRef->FastenerGridRef) SubsidenceOperatorRef->FastenerGridRef = FastenerGridRef;
		UE_LOG(LogHSRTrackDynamics, Log, TEXT("Subsidence operator linked"));
	}

	if (!DerailmentAssessor)
	{
		DerailmentAssessor = NewObject<UDerailmentAssessor>(this, TEXT("DerailmentAssessor"));
		DerailmentAssessor->RegisterComponent();
		DerailmentAssessor->StaticWheelLoad = TrainVehicleRef ? TrainVehicleRef->AxleLoad / 2.0f : 8500.0f;
		DerailmentAssessor->NumWheels = TrainVehicleRef ? 8 : 8;
		UE_LOG(LogHSRTrackDynamics, Log, TEXT("Derailment assessor initialized"));
	}

	if (TrackGeneratorRef && FastenerGridRef)
	{
		FastenerGridRef->InitializeFastenerGrid(TrackGeneratorRef);
		UE_LOG(LogHSRTrackDynamics, Log, TEXT("Fastener grid bound to track: %d fasteners"),
			FastenerGridRef->GetFastenerCount());
	}

	if (TrainVehicleRef)
	{
		if (bUseHighSpeedRK4Mode)
		{
			TrainVehicleRef->bUseRK4Integration = true;
			TrainVehicleRef->bDisableChaosPhysics = bAutoDisableChaosPhysics;
			TrainVehicleRef->RK4BaseSubstep = RK4BaseSubstep;
			TrainVehicleRef->RK4MinSubstep = RK4MinSubstep;
			TrainVehicleRef->RK4MaxSubstepsPerFrame = RK4MaxSubstepsPerFrame;
			TrainVehicleRef->bAdaptiveSubstepping = bAdaptiveSubstep;
			TrainVehicleRef->MaxEnergyGainPerSubstep = MaxEnergyGainPerStep;
			TrainVehicleRef->SpeedLimitKmh = SpeedLimitKmh;
			TrainVehicleRef->CurrentStiffnessScale = ContactStiffnessScaling;

			if (!NumericalStabilizer)
			{
				NumericalStabilizer = NewObject<UNumericalStabilizer>(this, TEXT("GlobalNumericalStabilizer"));
				NumericalStabilizer->RegisterComponent();
			}
			NumericalStabilizer->StiffnessConfig.MinStepsPerPeriod = MinStepsPerPeriod;
			NumericalStabilizer->StiffnessConfig.SafetyFactor = StabilitySafetyFactor;
			NumericalStabilizer->StiffnessConfig.bEnableEnergyPreservation = bEnableEnergyCorrection;
			NumericalStabilizer->StiffnessConfig.MaxEnergyGainPerStep = MaxEnergyGainPerStep;
		}

		TrainVehicleRef->InitializeVehicle();
		UE_LOG(LogHSRTrackDynamics, Log, TEXT("Train vehicle initialized at speed %.1f m/s"),
			TrainVehicleRef->TargetSpeed);
	}

	if (bEnableIrregularity)
	{
		GenerateIrregularitySpectrum();
		UE_LOG(LogHSRTrackDynamics, Log, TEXT("Rail irregularity spectrum generated: %d samples"),
			IrregularitySpectrum.SpectrumSamples);
	}

	Diagnostics.ActiveFastenerCount = FastenerGridRef ? FastenerGridRef->GetFastenerCount() : 0;

	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Solver initialization complete. Substep: %.4f s, Max substeps: %d"),
		PhysicsSubstepSize, MaxSubstepsPerFrame);
}

void ATrackDynamicsSolver::StartSimulation()
{
	bSimulationRunning = true;
	AccumulatedTime = 0.0f;
	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Simulation started"));
}

void ATrackDynamicsSolver::StopSimulation()
{
	bSimulationRunning = false;
	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Simulation stopped at t=%.3f s"), Diagnostics.CurrentSimTime);
}

void ATrackDynamicsSolver::ResetSimulation()
{
	bSimulationRunning = false;
	Diagnostics = FSimulationDiagnostics();
	AccumulatedTime = 0.0f;

	if (TrainVehicleRef)
	{
		TrainVehicleRef->InitializeVehicle();
	}
	if (FastenerGridRef && TrackGeneratorRef)
	{
		FastenerGridRef->InitializeFastenerGrid(TrackGeneratorRef);
	}

	UE_LOG(LogHSRTrackDynamics, Log, TEXT("Simulation reset"));
}

void ATrackDynamicsSolver::ExecutePhysicsSubstep(float Dt)
{
	AccumulatedTime += Dt;
	Diagnostics.CurrentSimTime = AccumulatedTime;
	Diagnostics.SubstepCount++;

	if (bEnableVehicleDynamics && TrainVehicleRef)
	{
		TrainVehicleRef->StepMBSDynamics(Dt);
	}

	if (bEnableTrackDynamics && FastenerGridRef && !FastenerGridRef->bUseUEPhysicsConstraints)
	{
		FastenerGridRef->UpdateCustomSpringForces(Dt);
	}

	if (bRunCoupledSimulation)
	{
		CoupleWheelToTrack();
	}
}

void ATrackDynamicsSolver::CoupleWheelToTrack()
{
	if (!TrainVehicleRef || !FastenerGridRef) return;

	for (int32 i = 0; i < TrainVehicleRef->WheelContactResults.Num(); ++i)
	{
		FContactPatchResult& Contact = TrainVehicleRef->WheelContactResults[i];
		if (!Contact.bInContact) continue;

		FVector WheelPos = Contact.ContactPointWorld;
		int32 NearestFastener = FastenerGridRef->FindNearestFastenerIndex(WheelPos);

		if (NearestFastener >= 0)
		{
			float WheelLoad = Contact.NormalForce;
			FastenerGridRef->ApplyWheelLoadToRail(NearestFastener, WheelLoad);

			FVector RailDisp = FastenerGridRef->GetRailDisplacementAtPosition(WheelPos);

			if (TrainVehicleRef->WheelRailContacts.IsValidIndex(i))
			{
				UWheelRailContact* WRC = TrainVehicleRef->WheelRailContacts[i];
				if (WRC)
				{
					WRC->RailIrregularityVertical = GetRailIrregularityAt(WheelPos.X, true);
					WRC->RailIrregularityLateral = GetRailIrregularityAt(WheelPos.X, false);
				}
			}
		}
	}
}

void ATrackDynamicsSolver::GenerateRailIrregularity()
{
	GenerateIrregularitySpectrum();
}

void ATrackDynamicsSolver::GenerateIrregularitySpectrum()
{
	RandomStream.Initialize(IrregularitySpectrum.RandomSeed);

	int32 N = IrregularitySpectrum.SpectrumSamples;
	VerticalIrregularitySpectrumMag.SetNum(N);
	LateralIrregularitySpectrumMag.SetNum(N);
	VerticalIrregularityPhase.SetNum(N);
	LateralIrregularityPhase.SetNum(N);

	float MaxOmega = 2.0f * PI / IrregularitySpectrum.MinWavelength;
	float MinOmega = 2.0f * PI / IrregularitySpectrum.MaxWavelength;
	float DOmega = (MaxOmega - MinOmega) / N;

	for (int32 i = 0; i < N; ++i)
	{
		float Omega = MinOmega + i * DOmega;
		float Omega2 = Omega * Omega;

		float PSD_Vertical = IrregularitySpectrum.AWavelengthA /
			(Omega2 * Omega2 + IrregularitySpectrum.AWavelengthB);
		float PSD_Lateral = IrregularitySpectrum.CWavelengthA /
			(Omega2 * Omega2 + IrregularitySpectrum.CWavelengthB);

		float dOmega = DOmega;
		VerticalIrregularitySpectrumMag[i] = FMath::Sqrt(2.0f * PSD_Vertical * dOmega);
		LateralIrregularitySpectrumMag[i] = FMath::Sqrt(2.0f * PSD_Lateral * dOmega);

		VerticalIrregularityPhase[i] = RandomStream.FRandRange(0.0f, 2.0f * PI);
		LateralIrregularityPhase[i] = RandomStream.FRandRange(0.0f, 2.0f * PI);
	}
}

float ATrackDynamicsSolver::GetRailIrregularityAt(float DistanceAlongTrack, bool bVertical) const
{
	if (!bEnableIrregularity || VerticalIrregularitySpectrumMag.Num() == 0) return 0.0f;

	int32 N = VerticalIrregularitySpectrumMag.Num();
	float MaxOmega = 2.0f * PI / IrregularitySpectrum.MinWavelength;
	float MinOmega = 2.0f * PI / IrregularitySpectrum.MaxWavelength;
	float DOmega = (MaxOmega - MinOmega) / N;

	float Irregularity = 0.0f;

	for (int32 i = 0; i < N; ++i)
	{
		float Omega = MinOmega + i * DOmega;
		float Phase = bVertical ? VerticalIrregularityPhase[i] : LateralIrregularityPhase[i];
		float Mag = bVertical ? VerticalIrregularitySpectrumMag[i] : LateralIrregularitySpectrumMag[i];

		Irregularity += Mag * FMath::Cos(Omega * DistanceAlongTrack + Phase);
	}

	return Irregularity;
}

void ATrackDynamicsSolver::GetSimulationStats(FSimulationDiagnostics& OutStats) const
{
	OutStats = Diagnostics;
}

void ATrackDynamicsSolver::UpdateDiagnostics(float DeltaTime)
{
	Diagnostics.RealTimeFactor = (DeltaTime > KINDA_SMALL_NUMBER) ?
		(PhysicsSubstepSize / DeltaTime) : 0.0f;

	Diagnostics.MaxWheelForce = 0.0f;
	Diagnostics.MaxContactPressure = 0.0f;
	Diagnostics.MaxLateralCreepage = 0.0f;

	if (TrainVehicleRef)
	{
		Diagnostics.CurrentVehicleSpeed = TrainVehicleRef->GetCurrentSpeed();

		for (const auto& Contact : TrainVehicleRef->WheelContactResults)
		{
			float ForceMag = Contact.TangentialForce.Size() + FMath::Abs(Contact.NormalForce);
			Diagnostics.MaxWheelForce = FMath::Max(Diagnostics.MaxWheelForce, ForceMag);
			Diagnostics.MaxContactPressure = FMath::Max(Diagnostics.MaxContactPressure, Contact.MaxContactPressure);
		}

		for (const auto& WRC : TrainVehicleRef->WheelRailContacts)
		{
			if (WRC)
			{
				Diagnostics.MaxLateralCreepage = FMath::Max(
					Diagnostics.MaxLateralCreepage,
					FMath::Abs(WRC->CurrentCreepage.LateralCreepage));
			}
		}

		if (bUseHighSpeedRK4Mode)
		{
			Diagnostics.TotalRK4Substeps = TrainVehicleRef->TotalSubstepsThisFrame;
			Diagnostics.EnergyCorrections = TrainVehicleRef->EnergyCorrectionCount;
			Diagnostics.VelocityClamps = TrainVehicleRef->VelocityClampCount;
			Diagnostics.CurrentStiffnessScale = TrainVehicleRef->CurrentStiffnessScale;
			Diagnostics.CurrentDampingScale = TrainVehicleRef->CurrentDampingScale;
			Diagnostics.DominantFrequency = TrainVehicleRef->DominantFrequency;

			if (NumericalStabilizer && bEnableStabilityAnalysis)
			{
				FStabilityAnalysisResult Stability = NumericalStabilizer->AnalyzeStability(
					TrainVehicleRef->DominantFrequency,
					TrainVehicleRef->PrimarySuspension.AxleBoxSpringStiffnessZ,
					TrainVehicleRef->WheelsetMass,
					DeltaTime
				);
				Diagnostics.StabilityRatio = Stability.StabilityRatio;
				Diagnostics.StabilityCondition = Stability.Condition;

				if (Stability.Condition == EStabilityCondition::Unstable)
				{
					UE_LOG(LogHSRTrackDynamics, Error,
						TEXT("STABILITY FAILURE: ratio=%.3f, freq=%.1f Hz, dt=%.3e s"),
						Stability.StabilityRatio, Stability.HighestFrequency, Stability.CurrentDt);
				}
			}
		}
	}

	if (DerailmentAssessor && TrainVehicleRef)
	{
		DerailmentAssessor->CurrentSpeed = TrainVehicleRef->GetCurrentSpeed();

		int32 NumW = TrainVehicleRef->WheelContactResults.Num();
		DerailmentAssessor->WheelLateralForces.SetNum(NumW);
		DerailmentAssessor->WheelVerticalForces.SetNum(NumW);
		DerailmentAssessor->WheelVerticalAccelerations.SetNum(NumW);
		DerailmentAssessor->WheelLateralAccelerations.SetNum(NumW);

		for (int32 i = 0; i < NumW; ++i)
		{
			const FContactPatchResult& Contact = TrainVehicleRef->WheelContactResults[i];
			DerailmentAssessor->WheelLateralForces[i] = Contact.TangentialForce.Y;
			DerailmentAssessor->WheelVerticalForces[i] = Contact.NormalForce;
			DerailmentAssessor->WheelVerticalAccelerations[i] = 0.0f;
			DerailmentAssessor->WheelLateralAccelerations[i] = 0.0f;

			if (TrainVehicleRef->WheelRailContacts.IsValidIndex(i) && TrainVehicleRef->WheelRailContacts[i])
			{
				DerailmentAssessor->WheelVerticalAccelerations[i] = TrainVehicleRef->WheelRailContacts[i]->WheelVelocity.Z;
				DerailmentAssessor->WheelLateralAccelerations[i] = TrainVehicleRef->WheelRailContacts[i]->WheelVelocity.Y;
			}
		}

		if (TrainVehicleRef->bUseRK4Integration)
		{
			DerailmentAssessor->CarbodyVerticalAcceleration = TrainVehicleRef->RK4CarBodyState.AngularVelocity.X;
			DerailmentAssessor->CarbodyLateralAcceleration = TrainVehicleRef->RK4CarBodyState.Velocity.Y;
		}

		DerailmentAssessor->ComputeDerailmentAssessment();

		Diagnostics.DerailmentCoefficient = DerailmentAssessor->CurrentReport.MaxDerailmentCoefficient;
		Diagnostics.LoadReductionRatio = DerailmentAssessor->CurrentReport.MaxLoadReductionRatio;
		Diagnostics.PeakVerticalAccel = DerailmentAssessor->CurrentReport.PeakVerticalAcceleration;
		Diagnostics.PeakCarbodyVertAccel = DerailmentAssessor->CurrentReport.MaxCarbodyVerticalAccel;
		Diagnostics.bDerailmentImminent = DerailmentAssessor->CurrentReport.bDerailmentImminent;
	}

	if (SubsidenceOperatorRef)
	{
		Diagnostics.MaxSettlementDepth = SubsidenceOperatorRef->MaxSettlementDepth;
		Diagnostics.MaxStiffnessReduction = SubsidenceOperatorRef->MaxStiffnessReductionApplied;
	}

	Diagnostics.MaxRailDisplacement = 0.0f;
	if (FastenerGridRef)
	{
		for (const auto& State : FastenerGridRef->FastenerStates)
		{
			float DispMag = State.RailDisplacement.Size();
			Diagnostics.MaxRailDisplacement = FMath::Max(Diagnostics.MaxRailDisplacement, DispMag);
		}
		Diagnostics.ActiveFastenerCount = FastenerGridRef->GetFastenerCount();
	}

	float SubstepEnergy = 0.0f;
	if (TrainVehicleRef)
	{
		SubstepEnergy = TrainVehicleRef->GetCurrentSpeed() * Diagnostics.MaxWheelForce * DeltaTime * 0.001f;
	}
	Diagnostics.TotalEnergyDissipated += SubstepEnergy;

	if (FMath::Fmod(Diagnostics.CurrentSimTime, 1.0f) < DeltaTime)
	{
		LogDiagnostics();
	}
}

void ATrackDynamicsSolver::LogDiagnostics()
{
	if (bUseHighSpeedRK4Mode)
	{
		const TCHAR* StabilityStr = TEXT("Unknown");
		switch (Diagnostics.StabilityCondition)
		{
		case EStabilityCondition::Stable: StabilityStr = TEXT("Stable"); break;
		case EStabilityCondition::Warning: StabilityStr = TEXT("Warning"); break;
		case EStabilityCondition::Critical: StabilityStr = TEXT("Critical"); break;
		case EStabilityCondition::Unstable: StabilityStr = TEXT("UNSTABLE"); break;
		}

		UE_LOG(LogHSRTrackDynamics, Log,
			TEXT("t=%.2fs | V=%.1f km/h | RK4=%d | Q/P=%.3f | ΔP/P₀=%.1f%% | VertAccel=%.2fg | CarbodyVert=%.2fg | Settlement=%.1fmm | StiffReduction=%.1f%% | stability=%s"),
			Diagnostics.CurrentSimTime,
			Diagnostics.CurrentVehicleSpeed * 3.6f,
			Diagnostics.TotalRK4Substeps,
			Diagnostics.DerailmentCoefficient,
			Diagnostics.LoadReductionRatio * 100.0f,
			Diagnostics.PeakVerticalAccel / 9.81f,
			Diagnostics.PeakCarbodyVertAccel / 9.81f,
			Diagnostics.MaxSettlementDepth * 1000.0f,
			Diagnostics.MaxStiffnessReduction * 100.0f,
			StabilityStr);

		if (Diagnostics.bDerailmentImminent)
		{
			UE_LOG(LogHSRTrackDynamics, Error,
				TEXT("!!! DERAILMENT ALERT !!! Q/P=%.3f exceeds Nadal limit | ΔP/P₀=%.1f%% | Carbody VertAccel=%.2fg"),
				Diagnostics.DerailmentCoefficient,
				Diagnostics.LoadReductionRatio * 100.0f,
				Diagnostics.PeakCarbodyVertAccel / 9.81f);
		}
	}
	else
	{
		UE_LOG(LogHSRTrackDynamics, Log,
			TEXT("t=%.2fs | Speed=%.1f m/s | MaxForce=%.0f N | MaxPressure=%.1f MPa | MaxRailDisp=%.4f mm | Creepage=%.5f | Fasteners=%d"),
			Diagnostics.CurrentSimTime,
			Diagnostics.CurrentVehicleSpeed,
			Diagnostics.MaxWheelForce,
			Diagnostics.MaxContactPressure / 1.0e6f,
			Diagnostics.MaxRailDisplacement * 1000.0f,
			Diagnostics.MaxLateralCreepage,
			Diagnostics.ActiveFastenerCount);
	}
}
