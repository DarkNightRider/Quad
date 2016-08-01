
#include "Quad_PID.h"

using namespace Quad;
// Examples for _filter:
// f_cut = 10 Hz -> _filter = 15.9155e-3
// f_cut = 15 Hz -> _filter = 10.6103e-3
// f_cut = 20 Hz -> _filter =  7.9577e-3
// f_cut = 25 Hz -> _filter =  6.3662e-3
// f_cut = 30 Hz -> _filter =  5.3052e-3
const float  PID::_filter = 7.9577e-3; // �� "1 / ( 2 * PI * f_cut )"�����ʽ�������;


int32_t PID::get_p(int32_t error)
{
    return (float)error * kP;
}

int32_t PID::get_i(int32_t error, float dt)
{
    if((kI != 0) && (dt != 0)) {
        integrator += ((float)error * kI) * dt;
				//�����޷�
				integrator = constrain_float(integrator, -imax, +imax);		
				
        return integrator;
    }
    return 0;
}

void PID::reset_I(void)
{
	integrator = 0;
}

int32_t PID::get_d(int32_t error, float dt)
{
	if ((kD != 0) && (dt != 0)) {
			float derivative;
	if (isnan(last_derivative)) {		
		derivative = 0;
		last_derivative = 0;
	} else {
		derivative = (error - last_error) / dt;
	}

	// һ����ɢ��ͨ�˲��������͸�Ƶ�����Կ������ĸ���
	derivative = last_derivative +
								(dt / ( _filter + dt)) * (derivative - last_derivative);

	last_error	= error;
	last_derivative = derivative;

	return kD * derivative;
	}
    return 0;
}

int32_t PID::get_pi(int32_t error, float dt)
{
    return get_p(error) + get_i(error, dt);
}

int32_t PID::get_pid(int32_t error, float dt)
{
    return get_p(error) + get_i(error, dt) + get_d(error, dt);
}



