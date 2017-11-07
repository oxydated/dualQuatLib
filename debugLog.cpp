#include <string>
#include <sstream>
#include <Windows.h>
#include "debugLog.h"

namespace {
	std::wstring outStr;
	std::wstring resultString;
	std::wostringstream outStream(outStr);
}

namespace oxyde {
	namespace log {
		void printDualQuat(std::wstring name, const float* dualQuat)
		{
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << name << " = {";
			outStream << dualQuat[0] << "," << dualQuat[1] << "," << dualQuat[2] << "," << dualQuat[3] << ",";
			outStream << dualQuat[4] << "," << dualQuat[5] << "," << dualQuat[6] << "," << dualQuat[7];
			outStream << "}.dualQuatUnit";
			outStream << std::endl;

			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printDualQuat(std::wstring name, const float qs, const float qx, const float qy, const float qz, const float dqs, const float dqx, const float dqy, const float dqz )
		{
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << name << " = {";
			outStream << qs << "," << qx << "," << qy << "," << qz << ",";
			outStream << dqs << "," << dqx << "," << dqy << "," << dqz;
			outStream << "}.dualQuatUnit";
			outStream << std::endl;

			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printDualQuatParameters(std::wstring name, 
			const float nx, const float ny, const float nz,
			const float angle, const float slide,
			const float mx, const float my, const float mz)
		{
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << name << " = N[makeDualVersor[" << angle << ", {" << nx << "," << ny << "," << nz << " }, " << slide << ", {" << mx << "," << my << "," << mz << " }]]" << std::endl;
			outStream << std::endl;

			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printMatrix(std::wstring name, const float* m)
		{
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << name << " = Transpose[{";
			outStream << "{" << m[0] << ", " << m[1] << ", " << m[2] << ", " << m[3] << "}" << ",";
			outStream << "{" << m[4] << ", " << m[5] << ", " << m[6] << ", " << m[7] << "}" << ",";
			outStream << "{" << m[8] << ", " << m[9] << ", " << m[10] << ", " << m[11] << "}" << ",";
			outStream << "{" << m[12] << ", " << m[13] << ", " << m[14] << ", " << m[15] << "}" << "}]";
			outStream << std::endl;

			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printPointInSpace(std::wstring name, const float px, const float py, const float pz)
		{
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << name << " = {";
			outStream << px << "," << py << "," << pz;
			outStream << "}";
			outStream << std::endl;

			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printNamedParameter(std::wstring name, std::wstring wstr)
		{
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << name << " = "<< wstr << std::endl;
			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printNamedParameter(std::wstring name, float par)
		{
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << name << " = " << par << std::endl;
			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printLine() {
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << "______________________________________________________" << std::endl;
			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printText(std::string str) {
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << str.c_str() << std::endl;
			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}

		void printText(std::wstring wstr) {
			outStream.str(outStr);
			outStream << std::fixed;
			outStream << wstr << std::endl;
			resultString = outStream.str();
			OutputDebugString(resultString.c_str());
		}
		
	}
}
