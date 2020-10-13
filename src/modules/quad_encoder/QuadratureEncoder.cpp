#include "QuadratureEncoder.hpp"

// extern "C" __EXPORT int quad_encoder_main(int argc, char *argv[]);

QuadratureEncoder::QuadratureEncoder(int A,int B):EncA(A), EncB(B){
	px4_arch_gpiosetevent(EncA, true, true, true, &QuadratureEncoder::EncAInterruptCallback, this);
	px4_arch_gpiosetevent(EncB, true, true, true, &QuadratureEncoder::EncBInterruptCallback, this);
	// cpr = _param_encoder_cpr.get();

}

int QuadratureEncoder::EncAInterruptCallback(int irq, void *context, void *arg)
{
	QuadratureEncoder *enc = static_cast<QuadratureEncoder *>(arg);
	if(px4_arch_gpioread(enc->EncA) != px4_arch_gpioread(enc->EncB))
		enc->count ++;
	else
		enc->count --;
	return 0;
}

int QuadratureEncoder::EncBInterruptCallback(int irq, void *context, void *arg)
{
	QuadratureEncoder *enc = static_cast<QuadratureEncoder *>(arg);
	if(px4_arch_gpioread(enc->EncB) == px4_arch_gpioread(enc->EncA))
		enc->count ++;
	else
		enc->count --;
	return 0;
}

void QuadratureEncoder::calcVelocity(void *arg)
{
	QuadratureEncoder *enc = static_cast<QuadratureEncoder *>(arg);
	hrt_abstime time = hrt_absolute_time();

	int32_t count = enc->count;
	uint32_t fwd_diff = count - enc->lastCount;
	uint32_t rev_diff = enc->lastCount - count;
	// At this point, abs(diff) is always <= 2^31, so this cast from unsigned to signed is safe.
	int32_t diff = fwd_diff <= rev_diff ? fwd_diff : -int32_t(rev_diff);

	enc->lastCount = count;
	enc->speed = (diff / enc->cpr) * 2.0f * 3.14159f / ((time - enc->lastTime) / 1000000.0f);

	enc->lastTime = time;

	enc->updated = true;
}

// int quad_encoder_main(int argc, char *argv[])
// {
// 	QuadratureEncoder M1(M1A,M1B), M2(M2A,M2B);

// 	static struct hrt_call calc_vel1;
// 	static struct hrt_call calc_vel2;
// 	hrt_call_every(&calc_vel1, 10, 10000, QuadratureEncoder::calcVelocity, &M1);
// 	hrt_call_every(&calc_vel2, 10, 10000, QuadratureEncoder::calcVelocity, &M2);

// 	uORB::PublicationMulti<wheel_encoders_s> _wheelEncodersAdv[2] { ORB_ID(wheel_encoders), ORB_ID(wheel_encoders)};
// 	wheel_encoders_s _wheelEncoderMsg[2];


// 	// uint32_t _lastEncoderCount[2] {0, 0};
// 	// int64_t _encoderCounts[2] {0, 0};
// 	// float _motorSpeeds[2] {0.0f, 0.0f};

// 	while(true){
// 		if(M1.isUpdated() && M2.isUpdated())
// 		{
// 			// _lastEncoderCount[0] = M1.getLastCount();
// 			// _lastEncoderCount[1] = M2.getLastCount();
// 			_wheelEncoderMsg[0].timestamp = hrt_absolute_time();
// 			_wheelEncoderMsg[1].timestamp = hrt_absolute_time();


// 			_wheelEncoderMsg[0].encoder_position = M1.getCount();
// 			_wheelEncoderMsg[1].encoder_position = M2.getCount();

// 			_wheelEncoderMsg[0].speed = M1.getVelocity();
// 			_wheelEncoderMsg[1].speed = M2.getVelocity();

// 			_wheelEncodersAdv[0].publish(_wheelEncoderMsg[0]);
// 			_wheelEncodersAdv[1].publish(_wheelEncoderMsg[1]);

// 		}
// 		else{
// 			usleep(10000);
// 		}
// 	}

// 	return 0;
// }
