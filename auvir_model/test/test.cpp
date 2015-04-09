#include <iostream>
#include "gtest/gtest.h"

TEST(checktest, checktest)
{
    const std::string s1 = "test string";
    const std::string s2 = "test string 2";
    EXPECT_EQ(s1, s2);
}

/*
int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return ::RUN_ALL_TESTS();
}
*/

