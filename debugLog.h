#pragma once
#include <string>

namespace oxyde {
	namespace log {
		void printDualQuat(std::wstring name, const float* dualQuat);

		void printDualQuat(std::wstring name, const float qs, const float qx, const float qy, const float qz, const float dqs, const float dqx, const float dqy, const float dqz);

		void printDualQuatParameters(std::wstring name,
			const float nx, const float ny, const float nz,
			const float angle, const float slide,
			const float mx, const float my, const float mz);

		void printMatrix(std::wstring name, const float* m);

		void printLine();

		void printText(std::string str);

		void printText(std::wstring wstr);

		void printNamedParameter(std::wstring name, std::wstring wstr);
	}
}
