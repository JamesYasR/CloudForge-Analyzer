#include "Basic/funcs.h"
#include <random>
#include <string>
#include <ctime>

std::string to_string_with_precision(double num, int precision) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << num;
    return stream.str();
}

std::string GenerateRandomName(const std::string& prefix) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<int> dis(1000, 9999);

    return prefix + "_" + std::to_string(dis(gen));
}