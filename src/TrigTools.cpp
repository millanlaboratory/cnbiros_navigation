#ifndef CNBIROS_CORE_TRIGTOOLS_CPP
#define CNBIROS_CORE_TRIGTOOLS_CPP

#include "cnbiros_navigation/TrigTools.hpp"

namespace cnbiros {
	namespace navigation {

float TrigTools::AngleNorm(float angle) {
	float nangle;
	nangle = fmod(angle + M_PI, 2 * M_PI);

	return nangle >=0 ? (nangle-M_PI) : (nangle + M_PI);
}

float TrigTools::AngleToDeg(float rad) {
	return TrigTools::AngleNorm(rad)*180.0f/M_PI;
}	

float TrigTools::AngleToRad(float deg) {
	return TrigTools::AngleNorm(deg*M_PI/180.0f);
}	

float TrigTools::Radius(float x, float y) {
	return std::sqrt(std::pow(x, 2) + std::pow(y, 2));
}

float TrigTools::Angle(float x, float y) {
	return std::atan2(y, x);
}

	}
}


#endif
