#ifndef HELPER_H
#define HELPER_H

#include <math.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

inline void updateTextFile(ofstream &outfile, vector<double> vals) {
    std::ofstream outfile;
    outfile.open(path1,std::ios_base::app);
    //outfile<<"Timestamp: "<< timeSTP<<endl;
    //outfile<<"-----------------------------"<<endl;
    for(unsigned int i = 0; i < vals.size(); ++i) {
        outfile<<vals[i];
        if (i < vals.size()-1) {
            outfile<<" ";
        } else {
            outfile<<endl;
        }
    }
    //outfile<<"-----------------------------"<<endl;
    outfile.close();
}














#endif // end definition 