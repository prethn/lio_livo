#ifndef DEBUGOUT_HPP
#define DEBUGOUT_HPP

#include <iostream>

#define debugout true
#define ROSINFO1(a) if(debugout) do{std::cout << a << std::endl;} while (0)

#endif