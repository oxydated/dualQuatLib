#ifndef __DUALQUATERIONFUNCTIONS_H__
#define __DUALQUATERIONFUNCTIONS_H__

#include "dualQuaternionMath.h"

namespace oxyde {
	namespace DQ {

		struct dualQuatParameters {
			float qS;
			float theSin;
			float Ux, Uy, Uz;
			float theSfactor;
			float Mx, My, Mz;
		};

		//def dual_quat_transform_line(((qs, qx, qy, qz), (dqs, dqx, dqy, dqz)), ((gs, gx, gy, gz), (dgs, dgx, dgy, dgz))) :
		void dual_quat_transform_line(DUALQUAARG(q),
			DUALQUAARG(g),
			DUALQUAARG(o));

		//def dual_quat_transform_point(((qs, qx, qy, qz), (dqs, dqx, dqy, dqz)), ((gs, gx, gy, gz), (dgs, dgx, dgy, dgz))) :
		void dual_quat_transform_point(DUALQUAARG(q),
			DUALQUAARG(g),
			DUALQUAARG(o));

		//def make_dual_vector(vecU, vecX) :
		void make_dual_vector(float Ux, float Uy, float Uz, float Vx, float Vy, float Vz, float& Nx, float& Ny, float& Nz, float& Mx, float& My, float& Mz);

		//def extractDualVersorParameters(someDualVersor) :
		void extractDualVersorParameters(DUALQUAARG(q),
			//return (qS, theSin), theUvector, theSfactor, theMvector
			//float& qS, float& theSin, float& Ux, float& Uy, float& Uz, float& theSfactor, float& Mx, float& My, float& Mz);
			dualQuatParameters& theParameters);

		void complementDualVersorParameters(const dualQuatParameters original, dualQuatParameters& complement);

		//def transformFromSourceToDestinationAxis(sourceTransform, destTransform) :
		void transformFromSourceToDestinationAxis(DUALQUAARG(source), DUALQUAARG(dest), DUALQUAARG(o));
		//	# transforming from Source to Destination
		//
		//	fromDestToSourceTransform = dual_quaternion_product(
		//															destTransform,
		//															dual_quaternion_conjugate(sourceTransform)
		//														)
		//	return fromDestToSourceTransform

		void extractRotationQuatFromDualQuat(DUALQUAARG(q), DUALQUAARG(o));

		void getVectorFromAxisToPoint(DUALQUAARG(q), float& px, float& py, float& pz, float& vx, float& vy, float& vz);
		
		void getVectorFromAxisToOrigim(DUALQUAARG(q), float& vx, float& vy, float& vz);

		void getSinAndCosFromPointAroundAxis(DUALQUAARG(q), float & px, float & py, float & pz, float &cosine, float &sine);

		void getAngleForPointAroundQuatAxis(DUALQUAARG(q), float & px, float & py, float & pz, float &angleForPoint, float &quatAngle);
	}
}

#endif