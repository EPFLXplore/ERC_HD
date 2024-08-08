#include <gtest/gtest.h>
#include "utils/expression_parser.h"

class ExpressionParserTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Setup code if necessary
    }

    void TearDown() override
    {
        // Cleanup code if necessary
    }
};

TEST_F(ExpressionParserTest, String2DoubleTest)
{
    ExpressionParser parser;
    EXPECT_DOUBLE_EQ(parser.string2double("3.14159*2"), 3.14159 * 2);
    EXPECT_DOUBLE_EQ(parser.string2double("6.0/2"), 3.0);
}

TEST_F(ExpressionParserTest, TokenizeTest)
{
    ExpressionParser parser;
    std::vector<std::string> tokens = parser.tokenize("3.14159*2/4");
    std::vector<std::string> expectedTokens = {"3.14159", "*", "2", "/", "4"};
    EXPECT_EQ(tokens, expectedTokens);
}

TEST_F(ExpressionParserTest, ParseTokensTest)
{
    ExpressionParser parser;
    std::vector<std::string> tokens = {"3.14159", "*", "2", "/", "4"};
    EXPECT_DOUBLE_EQ(parser.parseTokens(tokens), (3.14159 * 2) / 4);
}

TEST_F(ExpressionParserTest, ReplacePITest)
{
    ExpressionParser parser;
    std::string expression = "PI*2";
    parser.replacePI(expression);
    EXPECT_EQ(expression, std::to_string(M_PI) + "*2") << "Output: " << expression << ", Expected: 3.14159*2";
}
