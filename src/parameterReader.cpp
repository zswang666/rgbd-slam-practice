#include "parameterReader.h"

// contructor
ParameterReader::ParameterReader(string filename)
{
    ifstream fin(filename.c_str());
    if(!fin)
    {
        cerr << "parameter file does not exist." << endl;
        return;
    }
    while(!fin.eof())
    {
        // read a line
        string str;
        getline(fin, str);

        // ignore comment (start with '#')
        if(str[0] == '#')
            continue;

        // obtain parameter
        int pos = str.find("=");
        if(pos == -1)
            continue;
        string key = str.substr(0, pos); // str[0:pos] is key
        string value = str.substr(pos+1, str.length()); // str[pos+1:end] is value
        data[key] = value; // store to container

        if(!fin.good())
            continue;
    }
}

// getData
string ParameterReader::getData(string key)
{
    map<string, string>::iterator iter = data.find(key);
    if(iter == data.end())
    {
        cerr << "Parameter name " << key << " not found!" << endl;
        return string("NOT_FOUND");
    }
    return iter->second; // return value
}

