/****************************************************************
* Author :  xiihoo
* Website:  www.xiihoo.com
* E-mail :  robot4xiihoo@163.com
* Github :  https://github.com/xiihoo/Books_Robot_SLAM_Navigation
****************************************************************/

#ifndef FOO_H_
#define FOO_H_

#include <string>

namespace foo
{
    class MyPrint
    {
    public:
        MyPrint(std::string output);
        void ExcutePrint();

        std::string output_;
    };
}
#endif

