/*
 * =====================================================================================
 *
 *       Filename:  testbed.hpp
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  02.04.2015 17:39:27
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include <boost/filesystem.hpp>
#include "opencv/cv.hpp"

void test_function();

struct chunk2048
{
    uint8_t _1;
    uint8_t _2;
    uint8_t _3;
    uint8_t _4;
    uint8_t _5;
    uint8_t _6;
    uint8_t _7;
    uint8_t _8;
    uint8_t _data;
};

struct BinaryImage2048
{
    chunk2048 _array[150];
};

struct BinaryImage128
{
   uint8_t _data[2400];
};


void convert_image(cv::Mat1b& in, BinaryImage2048& out);
void convert_image(cv::Mat1b& in, BinaryImage128& out);

