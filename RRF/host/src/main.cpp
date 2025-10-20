#include <iostream>
#include <Version.h>

int main() {
	std::cout << "RRF host bootstrap build\n";
	std::cout << "Version: " << VERSION << "\n";
	std::cout << "Build date: " << DateText << TimeSuffix << "\n";
	return 0;
}
