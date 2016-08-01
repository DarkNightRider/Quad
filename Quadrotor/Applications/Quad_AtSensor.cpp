#include "Quad_AtSensor.h"


using namespace Quad;


int8_t AtSensor::updateMagn()
{
	return 0;
}

Vector3f AtSensor::getMagn()
{
	Vector3f r;
	r.x = 0.0;
	r.y = 0.0;
	r.z = 0.0;
	return r;
}