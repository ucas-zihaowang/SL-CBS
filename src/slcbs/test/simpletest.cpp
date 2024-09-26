#include <iostream>
#include <thread>
#include <functional>

#include "common.h"
// class MyClass {
// public:
//     void threadFunction(int param, int param2) {

//         std::cout << "Thread ID: " << std::this_thread::get_id() << ", Param: " << param << ", Param2: " << param2 << std::endl;
//     }

//     void createThreads() {

//         std::thread t1(&MyClass::threadFunction, this, 1, 4);
//         std::thread t2(&MyClass::threadFunction, this, 2, 5);
//         std::thread t3(&MyClass::threadFunction, this, 3, 6);


//         t1.join();
//         t2.join();
//         t3.join();
//     }
// };

// int main() {
//     MyClass obj;
//     obj.createThreads();

//     return 0;
// }

int main() {
    Eigen::VectorXd coeffs{Vec11f::Zero()};
    coeffs << 0, 0, 0, 0, 0, 0, 0.5 ,-5 , 18.75 , -11 , 1.6525;
    std::cout << coeffs.transpose() << std::endl;
    int num = RootFinder::countRoots( coeffs, 0, 0.5 ) ;
    std::cout << "root num: " <<  num << std::endl;

    return 0;
}