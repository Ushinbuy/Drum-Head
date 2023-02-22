//============================================================================
// Name        : LearningCpp.cpp
// Author      : Me
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "CppUTest/CommandLineTestRunner.h"
#include "memmoryManager.h"

using namespace std;

int main(int ac, char** av) {
	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

	eraseSector();

	initMemory(0);

	changeValueInFlash();

	return 0;

	// for tests uncomment this
//	return RUN_ALL_TESTS(ac, av);
}
