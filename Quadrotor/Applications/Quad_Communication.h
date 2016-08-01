#pragma once

namespace Quad
{
	class Communication
	{
	public:
		virtual void Init() = 0;
		virtual int TransmitData(uint8_t * data, int length) = 0;
		virtual int ReceiveData(uint8_t * data, int length) = 0;
		virtual void CheckEvent(void) = 0;
	};

	
	
};