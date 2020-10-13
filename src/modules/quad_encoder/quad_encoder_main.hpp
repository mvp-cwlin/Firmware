#pragma once

#include "QuadratureEncoder.hpp"
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>

// #define M1A (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTE | GPIO_PIN11)  //FMU CH3  TIM1CH1
// #define M1B (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTE | GPIO_PIN9) //FMU CH4  TIM1CH2
// #define M2A (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTD | GPIO_PIN13) //FMU CH5  TIM4CH2
// #define M2B (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTD | GPIO_PIN14) //FMU CH6  TIM4CH3
// #define M3A (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTA | GPIO_PIN5)  //FMU CAP1 TIM2CH1
// #define M3B (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTB | GPIO_PIN3)  //FMU CAP2 TIM2CH2

extern "C" __EXPORT int quad_encoder_main(int argc, char *argv[]);


class Encoders : public ModuleBase<Encoders>, public ModuleParams
{
public:
	Encoders();

	virtual ~Encoders() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Encoders *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update(bool force = false);




	// DEFINE_PARAMETERS(
	// 	(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
	// 	(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	// )

	// Subscriptions
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};

	const static int32_t M1A{(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTE | GPIO_PIN11)}; //FMU CH3  TIM1CH1
	const static int32_t M1B{(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTE | GPIO_PIN9)}; //FMU CH4  TIM1CH2
	const static int32_t M2A{(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTD | GPIO_PIN13)}; //FMU CH5  TIM4CH2
	const static int32_t M2B{(GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI | GPIO_PORTD | GPIO_PIN14)}; //FMU CH6  TIM4CH3

	QuadratureEncoder M1{M1A,M1B}, M2{M2A,M2B};

	uORB::PublicationMulti<wheel_encoders_s> _wheelEncodersAdv[2]{ ORB_ID(wheel_encoders), ORB_ID(wheel_encoders)};
	wheel_encoders_s _wheelEncoderMsg[2];




};

