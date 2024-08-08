#ifndef EXPRESSION_PARSER_H
#define EXPRESSION_PARSER_H

#include <vector>
#include <string>
#include <math.h>

class ExpressionParser
{
public:
    ExpressionParser() = default;

    double string2double(const std::string &expression);
    void replacePI(std::string &str);

    std::vector<std::string> tokenize(const std::string &expression);
    double parseTokens(const std::vector<std::string> &tokens);
};

#endif // EXPRESSION_PARSER_H
