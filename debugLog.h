#pragma once
#include <string>
#include <complex>

namespace oxyde {
	namespace log {

		using complex = std::complex<double>;

		void printDualQuat(std::wstring name, const float* dualQuat);

		void printDualQuat(std::wstring name, const float qs, const float qx, const float qy, const float qz, const float dqs, const float dqx, const float dqy, const float dqz);

		void printDualQuatParameters(std::wstring name,
			const float nx, const float ny, const float nz,
			const float angle, const float slide,
			const float mx, const float my, const float mz);

		void printMatrix(std::wstring name, const float* m);

		void printMatrix(std::wstring name, const double* m);

		void printPointInSpace(std::wstring name, const float px, const float py, const float pz);

		void printLine();

		void printText(std::string str);

		void printText(std::wstring wstr);

		void printNamedParameter(std::wstring name, std::wstring wstr);

		void printNamedParameter(std::wstring name, float par);

		void printNamedParameter(std::wstring name, int par);

		void printNamedParameter(std::wstring name, complex par);

		void printComplexVector(std::wstring name, const complex px, const complex py, const complex pz);

		void print3x3ComplexMatrix(std::wstring name, const complex m[]);

		std::wstring complexToWstr(complex num);
	}
}
