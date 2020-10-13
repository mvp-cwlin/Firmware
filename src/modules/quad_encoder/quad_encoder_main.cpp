#include "quad_encoder_main.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>


int Encoders::print_status()
{
	PX4_INFO("Running");
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int Encoders::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int Encoders::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("module",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

Encoders *Encoders::instantiate(int argc, char *argv[])
{

	Encoders *instance = new Encoders();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

Encoders::Encoders()
	: ModuleParams(nullptr)
{
}

void Encoders::run()
{
	// initialize parameters
	parameters_update(true);
	static struct hrt_call calc_vel1;
	static struct hrt_call calc_vel2;
	hrt_call_every(&calc_vel1, 10, 10000, QuadratureEncoder::calcVelocity, &M1);
	hrt_call_every(&calc_vel2, 10, 10000, QuadratureEncoder::calcVelocity, &M2);

	while (!should_exit()) {
		if(M1.isUpdated() && M2.isUpdated())
		{
			// _lastEncoderCount[0] = M1.getLastCount();
			// _lastEncoderCount[1] = M2.getLastCount();
			_wheelEncoderMsg[0].timestamp = hrt_absolute_time();
			_wheelEncoderMsg[1].timestamp = hrt_absolute_time();


			_wheelEncoderMsg[0].encoder_position = M1.getCount();
			_wheelEncoderMsg[1].encoder_position = M2.getCount();

			_wheelEncoderMsg[0].speed = M1.getVelocity();
			_wheelEncoderMsg[1].speed = M2.getVelocity();

			_wheelEncodersAdv[0].publish(_wheelEncoderMsg[0]);
			_wheelEncodersAdv[1].publish(_wheelEncoderMsg[1]);

		}
		else{
			usleep(10000);
		}
		parameters_update();
	}
}

void Encoders::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int Encoders::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a encoder module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "quad_encoder");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int quad_encoder_main(int argc, char *argv[])
{
	return Encoders::main(argc, argv);
}
