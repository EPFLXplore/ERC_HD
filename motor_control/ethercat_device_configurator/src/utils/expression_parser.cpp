#include "utils/expression_parser.h"

void ExpressionParser::replacePI(std::string &str)
{
    size_t pos;
    while ((pos = str.find("PI")) != std::string::npos)
    {
        str.replace(pos, 2, std::to_string(M_PI));
    }
}

std::vector<std::string> ExpressionParser::tokenize(const std::string &expression)
{
    size_t start_idx = 0;
    size_t end_idx = 0;
    std::vector<std::string> tokens;

    for (size_t i = 0; i < expression.size(); i++)
    {
        if (expression[i] == '*' || expression[i] == '/')
        {
            end_idx = i;
            std::string token = expression.substr(start_idx, end_idx - start_idx);
            tokens.push_back(token);
            tokens.push_back(std::string(1, expression[i]));
            start_idx = i + 1;
        }
    }
    tokens.push_back(expression.substr(start_idx, expression.size() - start_idx));
    return tokens;
}

double ExpressionParser::parseTokens(const std::vector<std::string> &tokens)
{
    double result = 1.0;
    char operation = '*';

    for (const auto &token : tokens)
    {
        if (token == "*")
        {
            operation = '*';
        }
        else if (token == "/")
        {
            operation = '/';
        }
        else
        {
            double value = std::stod(token);
            if (operation == '*')
            {
                result *= value;
            }
            else if (operation == '/')
            {
                result /= value;
            }
        }
    }
    return result;
}

double ExpressionParser::string2double(const std::string &expression)
{
    std::vector<std::string> tokens = tokenize(expression);
    return parseTokens(tokens);
}
