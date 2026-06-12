#include "HSRTrackDynamics.h"
#include "Modules/ModuleManager.h"

DEFINE_LOG_CATEGORY(LogHSRTrackDynamics);

class FHSRTrackDynamicsModule : public IModuleInterface
{
public:
	virtual void StartupModule() override
	{
		UE_LOG(LogHSRTrackDynamics, Log, TEXT("HSRTrackDynamics Module Started"));
	}

	virtual void ShutdownModule() override
	{
		UE_LOG(LogHSRTrackDynamics, Log, TEXT("HSRTrackDynamics Module Shutdown"));
	}
};

IMPLEMENT_MODULE(FHSRTrackDynamicsModule, HSRTrackDynamics)
