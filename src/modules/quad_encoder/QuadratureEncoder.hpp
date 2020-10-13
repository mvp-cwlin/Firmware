#include <lib/parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/parameter_update.h>

class QuadratureEncoder{
public:
	QuadratureEncoder():EncA(0), EncB(0){/*cpr = _param_encoder_cpr.get()*/};
	QuadratureEncoder(int A,int B);
	static int EncAInterruptCallback(int irq, void *context, void *arg);
	static int EncBInterruptCallback(int irq, void *context, void *arg);
	static void calcVelocity(void *arg);
	int getCount(){return count;};
	int getLastCount(){return lastCount;};
	void resetEncoders(){count = 0; lastCount = 0;};
	float getVelocity(){updated = false; return speed;};
	bool isUpdated(){return updated;};
	void run();
	void parameters_update(bool force);

private:
	static bool taskShouldExit;

	int32_t count{0};
	int32_t lastCount{0};
	const float cpr{979.62};
	float speed{0};
	const int EncA;
	const int EncB;
	bool updated{false};
	hrt_abstime lastTime{hrt_absolute_time()};

};
