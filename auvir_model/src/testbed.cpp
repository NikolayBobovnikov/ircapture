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
    std::string s = boost::filesystem::current_path()::string();
    std::cout << "current_path: " << s << std::endl;
}

void convert_image(Mat1b& in, BinaryImage2048& out)
{    
}

void convert_image(Mat1b& in, BinaryImage128& out)
{
    
}

