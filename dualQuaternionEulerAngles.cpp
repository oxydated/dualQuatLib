#include "dualQuaternionEulerAngles.h"

namespace oxyde {
	namespace DQ {

		inline void halfCosAndSineForTheta(float thetaX, float thetaY, float thetaZ,
			float &cosHalfX, float &cosHalfY, float &cosHalfZ,
			float &sinHalfX, float &sinHalfY, float &sinHalfZ) {

			cosHalfX = std::cos(thetaX / 2.);
			cosHalfY = std::cos(thetaY / 2.);
			cosHalfZ = std::cos(thetaZ / 2.);
			sinHalfX = std::sin(thetaX / 2.);
			sinHalfY = std::sin(thetaY / 2.);
			sinHalfZ = std::sin(thetaZ / 2.);
		}

		void quatFromEULERXYZ(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = cosHalfX*cosHalfY*cosHalfZ + sinHalfX*sinHalfY*sinHalfZ;
			ox = cosHalfY*cosHalfZ*sinHalfX - cosHalfX*sinHalfY*sinHalfZ;
			oy = cosHalfX*cosHalfZ*sinHalfY + cosHalfY*sinHalfX*sinHalfZ;
			oz = -(cosHalfZ*sinHalfX*sinHalfY) + cosHalfX*cosHalfY*sinHalfZ;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void quatFromEULERXZY(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = cosHalfX*cosHalfY*cosHalfZ - sinHalfX*sinHalfY*sinHalfZ;
			ox = cosHalfY*cosHalfZ*sinHalfX + cosHalfX*sinHalfY*sinHalfZ;
			oy = cosHalfX*cosHalfZ*sinHalfY + cosHalfY*sinHalfX*sinHalfZ;
			oz = -(cosHalfZ*sinHalfX*sinHalfY) + cosHalfX*cosHalfY*sinHalfZ;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void quatFromEULERYZX(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = cosHalfX*cosHalfY*cosHalfZ + sinHalfX*sinHalfY*sinHalfZ;
			ox = cosHalfY*cosHalfZ*sinHalfX - cosHalfX*sinHalfY*sinHalfZ;
			oy = cosHalfX*cosHalfZ*sinHalfY - cosHalfY*sinHalfX*sinHalfZ;
			oz = cosHalfZ*sinHalfX*sinHalfY + cosHalfX*cosHalfY*sinHalfZ;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void quatFromEULERYXZ(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = cosHalfX*cosHalfY*cosHalfZ - sinHalfX*sinHalfY*sinHalfZ;
			ox = cosHalfY*cosHalfZ*sinHalfX - cosHalfX*sinHalfY*sinHalfZ;
			oy = cosHalfX*cosHalfZ*sinHalfY + cosHalfY*sinHalfX*sinHalfZ;
			oz = cosHalfZ*sinHalfX*sinHalfY + cosHalfX*cosHalfY*sinHalfZ;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void quatFromEULERZXY(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = cosHalfX*cosHalfY*cosHalfZ + sinHalfX*sinHalfY*sinHalfZ;
			ox = cosHalfY*cosHalfZ*sinHalfX + cosHalfX*sinHalfY*sinHalfZ;
			oy = cosHalfX*cosHalfZ*sinHalfY - cosHalfY*sinHalfX*sinHalfZ;
			oz = -(cosHalfZ*sinHalfX*sinHalfY) + cosHalfX*cosHalfY*sinHalfZ;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void quatFromEULERZYX(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = cosHalfX*cosHalfY*cosHalfZ - sinHalfX*sinHalfY*sinHalfZ;
			ox = cosHalfY*cosHalfZ*sinHalfX + cosHalfX*sinHalfY*sinHalfZ;
			oy = cosHalfX*cosHalfZ*sinHalfY - cosHalfY*sinHalfX*sinHalfZ;
			oz = cosHalfZ*sinHalfX*sinHalfY + cosHalfX*cosHalfY*sinHalfZ;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void quatFromEULERXYX(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = std::pow(cosHalfX, 2)*cosHalfY - cosHalfY*std::pow(sinHalfX, 2);
			ox = 2 * cosHalfX*cosHalfY*sinHalfX;
			oy = std::pow(cosHalfX, 2)*sinHalfY + std::pow(sinHalfX, 2)*sinHalfY;
			oz = 0;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void quatFromEULERYZY(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = std::pow(cosHalfY, 2)*cosHalfZ - cosHalfZ*std::pow(sinHalfY, 2);
			ox = 0;
			oy = 2 * cosHalfY*cosHalfZ*sinHalfY;
			oz = std::pow(cosHalfY, 2)*sinHalfZ + std::pow(sinHalfY, 2)*sinHalfZ;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void quatFromEULERZXZ(float thetaX, float thetaY, float thetaZ, DUALQUAARG(o)) {
			float cosHalfX, cosHalfY, cosHalfZ;
			float sinHalfX, sinHalfY, sinHalfZ;

			halfCosAndSineForTheta(thetaX, thetaY, thetaZ,
				cosHalfX, cosHalfY, cosHalfZ,
				sinHalfX, sinHalfY, sinHalfZ);

			os = cosHalfX*std::pow(cosHalfZ, 2) - cosHalfX*std::pow(sinHalfZ, 2);
			ox = std::pow(cosHalfZ, 2)*sinHalfX + sinHalfX*std::pow(sinHalfZ, 2);
			oy = 0;
			oz = 2 * cosHalfX*cosHalfZ*sinHalfZ;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		quatFromEulerFunc quatFromEulerFuncByOrder(eulerOrder order)
		{
			quatFromEulerFunc theFuncToReturn = nullptr;

			switch (order) {
			case eulerOrder::EULERTYPE_XYZ:
				theFuncToReturn = quatFromEULERXYZ;
				break;
			case eulerOrder::EULERTYPE_XZY:
				theFuncToReturn = quatFromEULERXZY;
				break;
			case eulerOrder::EULERTYPE_YZX:
				theFuncToReturn = quatFromEULERYZX;
				break;
			case eulerOrder::EULERTYPE_YXZ:
				theFuncToReturn = quatFromEULERYXZ;
				break;
			case eulerOrder::EULERTYPE_ZXY:
				theFuncToReturn = quatFromEULERZXY;
				break;
			case eulerOrder::EULERTYPE_ZYX:
				theFuncToReturn = quatFromEULERZYX;
				break;
			case eulerOrder::EULERTYPE_XYX:
				theFuncToReturn = quatFromEULERXYX;
				break;
			case eulerOrder::EULERTYPE_YZY:
				theFuncToReturn = quatFromEULERYZY;
				break;
			case eulerOrder::EULERTYPE_ZXZ:
				theFuncToReturn = quatFromEULERZXZ;
				break;
			}
			return theFuncToReturn;
		}

	}
}