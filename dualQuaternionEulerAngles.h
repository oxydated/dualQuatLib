#pragma once

#include "dualQuaternionFunctions.h"

namespace oxyde {
	namespace DQ {

		enum eulerOrder :int { EULERTYPE_XYZ = 0, EULERTYPE_XZY = 1, EULERTYPE_YZX = 2, EULERTYPE_YXZ = 3, EULERTYPE_ZXY = 4, EULERTYPE_ZYX = 5, EULERTYPE_XYX = 6, EULERTYPE_YZY = 7, EULERTYPE_ZXZ = 8 };

		typedef void (*quatFromEulerFunc) (float , float , float , DUALQUAARG(o));

		void quatFromEULERXYZ(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		void quatFromEULERXZY(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		void quatFromEULERYZX(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		void quatFromEULERYXZ(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		void quatFromEULERZXY(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		void quatFromEULERZYX(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		void quatFromEULERXYX(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		void quatFromEULERYZY(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		void quatFromEULERZXZ(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o));

		quatFromEulerFunc quatFromEulerFuncByOrder(eulerOrder order);

	}
}
