#ifndef MOTOR_CONFIG_LOADER_H
#define MOTOR_CONFIG_LOADER_H

#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "Motor.h"
#include "utils/expression_parser.h"

class MotorConfigLoader
{
public:
    MotorConfigLoader(const std::string &filePath);

    std::vector<Motor> loadDeviceConfigs();

private:
    std::string filePath;
    ExpressionParser parser;

    void replacePI(std::string &str, double PI);

    std::vector<std::string> tokenize(const std::string &expression);

    double parseTokens(std::vector<std::string> tokens);

    double string2double(const std::string &expression);
};

#endif // DEVICE_CONFIG_LOADER_H
