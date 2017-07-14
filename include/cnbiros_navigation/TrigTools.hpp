#ifndef CNBIROS_CORE_TRIGTOOLS_HPP
#define CNBIROS_CORE_TRIGTOOLS_HPP

#include <cmath>

namespace cnbiros {
	namespace navigation {

class TrigTools {
	public:
		TrigTools(void) {};
		~TrigTools(void) {};

		static float AngleNorm(float angle);
		static float AngleToDeg(float rad);
		static float AngleToRad(float def);

		static float Radius(float x, float y);
		static float Angle(float x, float y);

};


	}
}


#endif
