/*
 * =====================================================================================
 *
 *       Filename:  testbed.cpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  02.04.2015 17:46:42
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#include "testbed.hpp"

void test_function()
{
    std::string s = oost::filesystem::current_path();
    std::cout << "current_path: " << s << std::endl;
}

