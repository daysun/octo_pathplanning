#pragma once
#include<iostream>
#include<string>
#include <sstream>
#include<boost/format.hpp>
using namespace std;

static int binToDec(int k){
    ostringstream ss;
   ss<<k;
   string str = ss.str();
    unsigned int i = 0;
       const char *pch = str.c_str();
       while (*pch == '0' || *pch == '1') {
           i <<= 1;
           i |= *pch++ - '0';
       }
       return (int)i;
}

static int binToDec(string str){
    unsigned int i = 0;
       const char *pch = str.c_str();
       while (*pch == '0' || *pch == '1') {
           i <<= 1;
           i |= *pch++ - '0';
       }
       return (int)i;
}

static string intToString(int k){
    ostringstream ss;
   ss<<k;
   string str = ss.str();
   return str;
}

static string decToBinStr2(int n){
    string s;//result
    for(int a = n; a ;a = a/2)
    {
        s=s+(a%2?'1':'0');
    }
    std::reverse(s.begin(),s.end());
    return s;
}

//for example
//line 5: 0101 column 7:0111
//morton:        110111
//return dec:   55
static string countMorton(int a,int b){
    string line = decToBinStr2(a);
    string column = decToBinStr2(b);
    if( line.size()>column.size()){
        int num = line.size()-column.size();
        for(int i =0;i<num;i++){
            column.insert(0,"0");
        }
    }else if(line.size()<column.size()){
        int num = column.size()-line.size();
        for(int i =0;i<num;i++){
            line.insert(0,"0");
        }
    }
//   cout<<line<<","<<column<<endl;
    int size = line.size() + column.size();
    char * reverse = new char[size+1];
    int j  =0;
    for(int i = line.size()-1;i>=0;i--){
        reverse[j++] = column[i];
        reverse[j++] = line[i];
    }
    reverse[size] = '\0';
//     cout<<reverse<<endl;
    ostringstream ss;
   ss<<reverse;
   string str = ss.str();
    string result(str.rbegin(),str.rend());
// cout<<result<<endl;
    string res = intToString(binToDec(result));
    return  res;
}
