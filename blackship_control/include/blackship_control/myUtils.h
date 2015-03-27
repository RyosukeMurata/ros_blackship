
#ifndef MYUTILS_H
#define MYUTILS_H

namespace myUtils {
#ifndef M_PI
static const double M_PI = 3.14159265358979323846;
#endif

#ifndef DEG2RAD
static const double DEG2RAD = (3.14159265358979323846/180.0);
#endif

#ifndef RAD2DEG
static const double RAD2DEG = (180.0/3.14159265358979323846);
#endif

static double limitMaxMin(double data, double max, double min){
	return (data>max) ? max : (data<min) ? min : data;
}

}

#endif

