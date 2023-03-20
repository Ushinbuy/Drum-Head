//============================================================================
// Name        : LearningCpp.cpp
// Author      : Me
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

//#include <eepromManager.h>
//#include <MemmorySystem.h>
#include <iostream>
#include "CppUTest/CommandLineTestRunner.h"
#include "TestParent.h"

using namespace std;

int main(int ac, char** av) {
//	cout << "!!!Hello World!!!" << endl; // prints !!!Hello World!!!

//	mainWork();

//	eeprom_work();
	Child child1, child2, child3;
	ChildAnother newOne;

	Parent::printAll();
//		cout << "length is " << list.size() << endl;
	return 0;

	// for tests uncomment this
//	return RUN_ALL_TESTS(ac, av);
}
