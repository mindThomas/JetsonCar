/* Copyright (C) 2018-2019 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <signal.h>
#include <condition_variable>
#include <stdlib.h>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/format.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "omp.h"

#include <iostream>
#include <cstring>
#include <string>

int main(int argc, char** argv ) {    
    std::string argv_str(realpath(argv[0], 0));
    std::string base = argv_str.substr(0, argv_str.find_last_of("/"));

    int argIdx = 0;

    if (!argv[1]) {
        printf("Error: No arguments specified! Use -h for help.\n");
        exit(-1);
    }

    if (!std::string(argv[1]).compare("-h")) {
        std::cout << "Examples of usecases:" << std::endl;
        std::cout << " ./test -i <number>" << std::endl;
        std::cout << "    Parse and print an integer number" << std::endl << std::endl;

        std::cout << " ./test -f <number>" << std::endl;
        std::cout << "    Parse and print a floating number (double)" << std::endl << std::endl;
        exit(0);
    }

    if (!std::string(argv[1]).compare("-i"))
    {
        if (argc != 3) {
            printf("Error: Needs a numeric input!\n");
            exit(-1);
        }
        int number;
        std::istringstream ss1(argv[2]);
        if (!(ss1 >> number)) {
            printf("Error: Number could not be parsed!\n");
            exit(-1);
        }
        std::cout << "Parsed number: " << number << std::endl;
    }

    if (!std::string(argv[1]).compare("-f"))
    {
        if (argc != 3) {
            printf("Error: Needs a numeric input!\n");
            exit(-1);
        }
        double number;
        std::istringstream ss1(argv[2]);
        if (!(ss1 >> number)) {
            printf("Error: Number could not be parsed!\n");
            exit(-1);
        }
        std::cout << "Parsed number: " << number << std::endl;
    }

    return 0;
}
