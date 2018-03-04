#pragma once

#include <iostream>
#include <fstream>
#include <map>
using namespace std;

// class for reading parameters from .txt files
class ParameterReader
{
public:
    // constructor
    ParameterReader(string filename);
    // get value according to keys
    string getData(string key);

    // container to save parameters, data[key] = value
    map<string, string> data;
};

