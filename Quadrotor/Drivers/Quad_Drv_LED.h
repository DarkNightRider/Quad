#pragma once
#include "board.h"

namespace Quad
{
	class LED
	{

	public:
		
		static void Init(void);

		void ON(void);
		void OFF(void);

	};

	extern LED led;
}


