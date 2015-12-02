#include "mex.h"
#include <stdio.h>
#include <vector>
#include <cmath>
#include <ctime>

using namespace std;

#include "esc_vector2.h"
#include "esc_mine.h"
#include "esc_tanke.h"
#include "esc_tankfinder.h"

void mexFunction(int outCount,mxArray *outVars[],int inCount,const mxArray *inVars[]) {
    static vector<Mine> mineList = vector<Mine>();
    static vector<Tanke> tankList = vector<Tanke>();
    static bool firstRound = true;
            
    if (inCount < 5)
        mexErrMsgIdAndTxt("MATLAB:textfunc:inputArgumentNumber", "Not Enough input arguments!");
    if (!mxIsStruct(inVars[0]) || !mxIsStruct(inVars[1]))
        mexErrMsgIdAndTxt("MATLAB:textfunc:inputNotStruct", "First Parameter must be a structure");
    
    if (firstRound || mineList.size() != mxGetNumberOfElements(inVars[0]) || tankList.size() != mxGetNumberOfElements(inVars[1])) {
        mineList = Mine::parseMineStruct(inVars[0]);
        tankList = Tanke::parseTankeStruct(inVars[1]);
    }
    
    Vector2 mePos = Vector2(inVars[2]);
    Vector2 meGes = Vector2(inVars[3]);
    Vector2 enemyPos = Vector2(inVars[4]);
    Vector2 enemyGes = Vector2(inVars[5]);
    double dignoreTanke = *mxGetPr(inVars[6]);
    int ignoreTanke = (int)dignoreTanke;
    double ebenen = *mxGetPr(inVars[7]);
    
    /*for (int i=0; i<mineList.size(); i++) {
        mexPrintf("mine: r: %f, pos: [%f, %f]\n", mineList[i].radius, mineList[i].pos.x, mineList[i].pos.y);
    }*/
    
    clock_t start;
    double duration;
    start = clock();
    
    //prepare tank list
    vector<Tanke*> tankListP = vector<Tanke*>();
    for (int i=0; i < tankList.size(); i++) {
        if (ignoreTanke == i+1) {
            tankListP.push_back(0);
        } else {
            tankListP.push_back(&tankList[i]);
        }
	}
    
    //do calculations
    vector<int> wps = vector<int>();
    Tankfinder tf = Tankfinder(mineList, enemyPos, enemyGes);
    tf.findTanke(wps, 0, tankListP, mePos, meGes, ebenen);
    
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    //mexPrintf("TankList (%d nodes from %d) found in %f s\n", wps.size(), tankListP.size(), duration);
    
    outVars[0] = mxCreateCellMatrix(1, wps.size());
    for (int i=0; i < wps.size(); i++) {
        mxArray * tmp = mxCreateDoubleScalar((double)wps[i]);
        mxSetCell(outVars[0], i, tmp);
    }
    
    firstRound = false;
}