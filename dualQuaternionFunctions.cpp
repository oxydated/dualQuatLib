#include <cmath>
#include <complex>
#include "dualQuaternionFunctions.h"
#include "linAlg.h"
#include "debugLog.h"

namespace oxyde {
	namespace DQ {

		//def dual_quat_transform_line(((qs, qx, qy, qz), (dqs, dqx, dqy, dqz)), ((gs, gx, gy, gz), (dgs, dgx, dgy, dgz))) :
		void dual_quat_transform_line(DUALQUAARG(q),
			DUALQUAARG(g),
			DUALQUAARG(o)) {

			//theDualVersor = ((qs, qx, qy, qz), (dqs, dqx, dqy, dqz))
			//theDualVersorConj = dual_quaternion_conjugate(theDualVersor)
			DUALQUAVAR(conj);
			dual_quaternion_conjugate(DUALQUACOMP(q), DUALQUACOMP(conj));
			//lineQuat = ((gs, gx, gy, gz), (dgs, dgx, dgy, dgz))
			//X = dual_quaternion_product(lineQuat, theDualVersorConj)
			DUALQUAVAR(temp);
			dual_quaternion_product(DUALQUACOMP(g), DUALQUACOMP(conj), DUALQUACOMP(temp));
			//return dual_quaternion_product(theDualVersor, X)
			dual_quaternion_product(DUALQUACOMP(q), DUALQUACOMP(temp), DUALQUACOMP(o));
		}

		//def dual_quat_transform_point(((qs, qx, qy, qz), (dqs, dqx, dqy, dqz)), ((gs, gx, gy, gz), (dgs, dgx, dgy, dgz))) :
		void dual_quat_transform_point(DUALQUAARG(q),
			DUALQUAARG(g),
			DUALQUAARG(o)) {

			//theDualVersor = ((qs, qx, qy, qz), (dqs, dqx, dqy, dqz))
			//theDualVersorConj = dual_quaternion_conjugate(theDualVersor)
			DUALQUAVAR(conj);
			dual_quaternion_conjugate(DUALQUACOMP(q), DUALQUACOMP(conj));
			//theDualVersorEpsilon = dual_quaternion_epsilon(theDualVersor)
			DUALQUAVAR(epsi);
			dual_quaternion_epsilon(DUALQUACOMP(q), DUALQUACOMP(epsi));
			//pointQuat = ((gs, gx, gy, gz), (dgs, dgx, dgy, dgz))
			//X = dual_quaternion_product(pointQuat, theDualVersorConj)
			DUALQUAVAR(temp);
			dual_quaternion_product(DUALQUACOMP(g), DUALQUACOMP(conj), DUALQUACOMP(temp));
			//return dual_quaternion_product(theDualVersorEpsilon, X)
			dual_quaternion_product(DUALQUACOMP(epsi), DUALQUACOMP(temp), DUALQUACOMP(o));
		}

		//def dual_quat_screw_line(theta, (ux, uy, uz), s, (mx, my, mz), (vx, vy, vz), (rx, ry, rz)) :

		//theDualVersor = dual_Versor(theta, (ux, uy, uz), s, (mx, my, mz))
		//theDualVersorConj = dual_quaternion_conjugate(theDualVersor)
		//lineQuat = line_quaternion((vx, vy, vz), (rx, ry, rz))
		//return dual_quaternion_product(theDualVersor, dual_quaternion_product(lineQuat, theDualVersorConj))

		//def dual_quat_screw_point(theta, (ux, uy, uz), s, (mx, my, mz), (px, py, pz)) :

		//theDualVersor = dual_Versor(theta, (ux, uy, uz), s, (mx, my, mz))
		//theDualVersorConj = dual_quaternion_conjugate(theDualVersor)
		//theDualVersorEpsilon = dual_quaternion_epsilon(theDualVersor)
		//pointQuat = point_quaternion((px, py, pz))
		//return dual_quaternion_product(theDualVersorEpsilon, dual_quaternion_product(pointQuat, theDualVersorConj))

		//def make_dual_vector(vecU, vecX) :
		void make_dual_vector(float Ux, float Uy, float Uz, float Vx, float Vy, float Vz, float& Nx, float& Ny, float& Nz, float& Mx, float& My, float& Mz) {
			//nU = vecU / np.linalg.norm(vecU)
			float norm = sqrt((Ux*Ux) + (Uy*Uy) + (Uz*Uz));
			Nx = Ux / norm; Nx = Ux / norm; Nx = Ux / norm;
			//vecM = np.cross(nU, vecX)
			Mx = Ny*Vz - Nz*Vy;
			My = -Nx*Vz + Nz*Vx;
			Mz = Nx*Vy - Ny*Vx;
			//return nU, vecM
		}

		//def extractDualVersorParameters(someDualVersor) :
		void extractDualVersorParameters(DUALQUAARG(q),
			//return (qS, theSin), theUvector, theSfactor, theMvector
			//float& qS, float& theSin, float& Ux, float& Uy, float& Uz, float& theSfactor, float& Mx, float& My, float& Mz){
			dualQuatParameters& theParameters) {
			////((qS, qx, qy, qz), (dqS, dqx, dqy, dqz)) = someDualVersor
			//theParameters.qS = qs;

			////qV = np.array((qx, qy, qz))
			////dqV = np.array((dqx, dqy, dqz))
			////theSin = np.linalg.norm(qV)
			//theParameters.theSin = sqrt((qx*qx) + (qy*qy) + (qz*qz));

			////if (np.abs(qS)> 1.0) :
			//	//print "qS:", qS
			//	//if qS < 0.0 :
			//		//	qS = -1.0
			//	//else :
			//		//qS = 1.0
			//theParameters.qS = (theParameters.qS < -1.0) ? -1.0 :
			//	((theParameters.qS > 1.0) ? 1.0 : theParameters.qS);

			////if (np.abs(theSin)> 1.0) :
			////	print "theSin:", theSin
			////	if theSin < 0.0 :
			////		theSin = -1.0
			////	else :
			////	theSin = 1.0
			//theParameters.theSin = (theParameters.theSin < -1.0) ? -1.0 :
			//	((theParameters.theSin > 1.0) ? 1.0 : theParameters.theSin);

			////if np.abs(theSin) < 1.0e-7 :
			////	theUvector = np.array((1.0, 0.0, 0.0))
			////	theMvector = np.array((0.0, 0.0, 0.0))
			////	theSfactor = 0.0
			//if (fabs(theParameters.theSin) < 1.0e-7) {
			//	theParameters.Ux = 1.0; theParameters.Uy = 0.0; theParameters.Uz = 0.0;
			//	theParameters.Mx = .0; theParameters.My = 0.0; theParameters.Mz = 0.0;
			//	theParameters.theSfactor = 0.0;
			//}
			////else:
			//else {
			//	//theUvector = qV / theSin
			//	theParameters.Ux = qx / theParameters.theSin; theParameters.Uy = qy / theParameters.theSin; theParameters.Uz = qz / theParameters.theSin;

			//	//theMvector = (dqV + (qV*(1.0 / theSin))*qS*(dqS / theSin)) * (1.0 / theSin)
			//	theParameters.Mx = (dqx + (qx*(1.0 / theParameters.theSin))*theParameters.qS*(dqs / theParameters.theSin)) * (1.0 / theParameters.theSin);
			//	theParameters.My = (dqy + (qy*(1.0 / theParameters.theSin))*theParameters.qS*(dqs / theParameters.theSin)) * (1.0 / theParameters.theSin);
			//	theParameters.Mz = (dqz + (qz*(1.0 / theParameters.theSin))*theParameters.qS*(dqs / theParameters.theSin)) * (1.0 / theParameters.theSin);

			//	//theSfactor = dqS / (theSin / 2.0)
			//	theParameters.theSfactor = dqs / (theParameters.theSin / 2.0);

			//	//return (qS, theSin), theUvector, theSfactor, theMvector
			float cosHalf = qs;
			theParameters.qS = qs;

			if (std::abs(cosHalf - 1.0) < 0.00001) {
				theParameters.theSin = 0.;
				float dqxdqydqz[3] = { dqx, dqy, dqz };
				float normDqxDqyDqz = 0.;
				oxyde::linAlg::norm(dqxdqydqz, &normDqxDqyDqz);

				if (std::abs(normDqxDqyDqz) < 0.00001) {
					theParameters.Ux = 1.;
					theParameters.Uy = 0.;
					theParameters.Uz = 0.;

					theParameters.theSfactor = 0.;
				}
				else {
					theParameters.theSfactor = 2*normDqxDqyDqz;

					theParameters.Ux = -dqx/ normDqxDqyDqz;
					theParameters.Uy = -dqy / normDqxDqyDqz;
					theParameters.Uz = -dqz / normDqxDqyDqz;
				}

				theParameters.Mx = 0.;
				theParameters.My = 0.;
				theParameters.Mz = 0.;

				return;
			}

			float sinHalf = std::sqrt(std::pow(qx, 2) + std::pow(qy, 2) + std::pow(qz, 2));
			float nx = qx / sinHalf;
			float ny = qy / sinHalf;
			float nz = qz / sinHalf;
			float s = (2 * dqs) / sinHalf;
			float mx = (2 * dqx + nx*s*cosHalf) / (2.*sinHalf);
			float my = (2 * dqy + ny*s*cosHalf) / (2.*sinHalf);
			float mz = (2 * dqz + nz*s*cosHalf) / (2.*sinHalf);

			//theParameters.qS = cosHalf;
			theParameters.theSin = sinHalf;
			if (std::isnan<float>(nx) || std::isnan<float>(ny) || std::isnan<float>(nz)) {
				theParameters.Ux = 1.;
				theParameters.Uy = 0.;
				theParameters.Uz = 0.;
			}
			else {
				theParameters.Ux = nx;
				theParameters.Uy = ny;
				theParameters.Uz = nz;
			}

			theParameters.theSfactor = std::isnan<float>(s) ? 0. : s;

			if (std::isnan<float>(mx) || std::isnan<float>(my) || std::isnan<float>(mz)) {
				theParameters.Mx = 0.;
				theParameters.My = 0.;
				theParameters.Mz = 0.;
			}
			else {
				theParameters.Mx = mx;
				theParameters.My = my;
				theParameters.Mz = mz;
			}
		}

		void complementDualVersorParameters(const dualQuatParameters original, dualQuatParameters & complement)
		{
			complement.qS = -original.qS;
			complement.theSin = original.theSin;
				complement.Ux = -original.Ux;
				complement.Uy = -original.Uy;
				complement.Uz = -original.Uz;
				complement.theSfactor = -original.theSfactor;
				complement.Mx = -original.Mx;
				complement.My = -original.My;
				complement.Mz = -original.Mz;
		}

		//def transformFromSourceToDestinationAxis(sourceTransform, destTransform) :
		void transformFromSourceToDestinationAxis(DUALQUAARG(source), DUALQUAARG(dest), DUALQUAARG(o)) {

			//															dual_quaternion_conjugate(sourceTransform)

			DUALQUAVAR(conjSource);
			dual_quaternion_conjugate(DUALQUACOMP(source), DUALQUACOMP(conjSource));

			//	# transforming from Source to Destination
			//
			//	fromDestToSourceTransform = dual_quaternion_product(
			//															destTransform,
			//															dual_quaternion_conjugate(sourceTransform)
			//														)

			dual_quaternion_product(DUALQUACOMP(dest), DUALQUACOMP(conjSource), DUALQUACOMP(o));
			//dual_quaternion_product(DUALQUACOMP(conjSource), DUALQUACOMP(dest), DUALQUACOMP(o));
			//	return fromDestToSourceTransform
		}

		void extractRotationQuatFromDualQuat(DUALQUAARG(q), DUALQUAARG(o)) {
			os = qs;
			ox = qx;
			oy = qy;
			oz = qz;
			dos = 0;
			dox = 0;
			doy = 0;
			doz = 0;
		}

		void getVectorFromAxisToPoint(DUALQUAARG(q), float & px, float & py, float & pz, float& vx, float& vy, float& vz)
		{
			float vecPartsQuared = std::pow(qx, 2) + std::pow(qy, 2) + std::pow(qz, 2);
			vx = (dqz*qy - py*qx*qy + px*std::pow(qy, 2) - dqy*qz - pz*qx*qz + px*std::pow(qz, 2)) / vecPartsQuared;
			vy = (-(dqz*qx) - px*qx*qy + dqx*qz - pz*qy*qz + py*(std::pow(qx, 2) + std::pow(qz, 2))) / vecPartsQuared;
			vz = (dqy*qx - dqx*qy + pz*(std::pow(qx, 2) + std::pow(qy, 2)) - px*qx*qz - py*qy*qz) / vecPartsQuared;
		}

		void getVectorFromAxisToOrigim(DUALQUAARG(q), float& vx, float& vy, float& vz)
		{
			float vecPartsQuared = std::pow(qx, 2) + std::pow(qy, 2) + std::pow(qz, 2);
			vx = (dqz*qy - dqy*qz) / vecPartsQuared;
			vy = (-(dqz*qx) + dqx*qz) / vecPartsQuared;
			vz = (dqy*qx - dqx*qy) / vecPartsQuared;
		}

		void getSinAndCosFromPointAroundAxis(DUALQUAARG(q), float & px, float & py, float & pz, float &cosine, float &sine) {
			float vectorToPoint[3];
			getVectorFromAxisToPoint(DUALQUACOMP(q), px, py, pz, vectorToPoint[0], vectorToPoint[1], vectorToPoint[2]);
			float distanceToPoint = 0;
			oxyde::linAlg::norm(vectorToPoint, &distanceToPoint);
			float normalToPoint[3];
			oxyde::linAlg::normalizeVector(vectorToPoint, normalToPoint);

			//oxyde::log::printLine();
			//oxyde::log::printText(L"getSinAndCosFromPointAroundAxis");

			//oxyde::log::printPointInSpace(L"thePoint", px, py, pz);

			//oxyde::log::printDualQuat(L"theQuat", DUALQUACOMP(q));
			//oxyde::log::printPointInSpace(L"normalToPoint", normalToPoint[0], normalToPoint[1], normalToPoint[2]);
			//oxyde::log::printNamedParameter(L"distanceToPoint", distanceToPoint);


			float vectorToOrigin[3];
			getVectorFromAxisToOrigim(DUALQUACOMP(q), vectorToOrigin[0], vectorToOrigin[1], vectorToOrigin[2]);
			float distanceToOrigin = 0;
			oxyde::linAlg::norm(vectorToOrigin, &distanceToOrigin);
			float normalToOrigin[3];
			oxyde::linAlg::normalizeVector(vectorToOrigin, normalToOrigin);


			//oxyde::log::printPointInSpace(L"normalToOrigin", normalToOrigin[0], normalToOrigin[1], normalToOrigin[2]);
			//oxyde::log::printNamedParameter(L"distanceToOrigin", distanceToOrigin);

			float quatAxis[] = { qx, qy, qz };
			float quatNormal[3];
			oxyde::linAlg::normalizeVector(quatAxis, quatNormal);
			float quatSine = 0;
			oxyde::linAlg::norm(quatAxis, &quatSine);


			//oxyde::log::printPointInSpace(L"quatNormal", quatNormal[0], quatNormal[1], quatNormal[2]);


			float crossVector[3];
			oxyde::linAlg::vectorCrossProduct(normalToOrigin, normalToPoint, crossVector);
			float crossNormal[3];
			oxyde::linAlg::normalizeVector(crossVector, crossNormal);
			oxyde::linAlg::norm(crossVector, &sine);
			float dotAxes;
			oxyde::linAlg::vectorDotProduct(quatNormal, crossNormal, &dotAxes);
			sine = (dotAxes > 0.0) ? sine : -sine;
			
			oxyde::linAlg::vectorDotProduct(normalToOrigin, normalToPoint, &cosine);
			

			//oxyde::log::printPointInSpace(L"crossNormal", crossNormal[0], crossNormal[1], crossNormal[2]);
			//oxyde::log::printNamedParameter(L"cosine", cosine);
			//oxyde::log::printNamedParameter(L"sine", sine);

			//using complex = std::complex<double>;
			////complex euler(qs, quatSine);
			////float quatAngle = std::real(complex(0, -1)*(std::log(euler)))/2;
			//float quatAngle = std::real(complex(0, -1)*(std::log(complex(qs, quatSine)))) * 2;
			//float pointAngle = std::real(complex(0, -1)*(std::log(complex(cosine, sine))));

			//oxyde::log::printNamedParameter(L"quatAngle", std::to_wstring(quatAngle));
			//oxyde::log::printNamedParameter(L"pointAngle", std::to_wstring(pointAngle));
			
		}

		void getAngleForPointAroundQuatAxis(DUALQUAARG(q), float & px, float & py, float & pz, float &angleForPoint, float &quatAngle){

			float sine = 0, cosine = 0;
			getSinAndCosFromPointAroundAxis(DUALQUACOMP(q), px, py, pz, cosine, sine);
			
			float quatAxis[] = { qx, qy, qz };
			float quatSine = 0;
			oxyde::linAlg::norm(quatAxis, &quatSine);

			using complex = std::complex<double>;
			//complex euler(qs, quatSine);
			//float quatAngle = std::real(complex(0, -1)*(std::log(euler)))/2;
			quatAngle = std::real(complex(0, -1)*(std::log(complex(qs, quatSine)))) * 2;
			angleForPoint = std::real(complex(0, -1)*(std::log(complex(cosine, sine))));

			//oxyde::log::printNamedParameter(L"quatAngle", quatAngle);
			//oxyde::log::printNamedParameter(L"pointAngle", angleForPoint);

		}
	}
}