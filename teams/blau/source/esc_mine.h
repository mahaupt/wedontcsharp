class Mine {
public: 
    float radius;
    Vector2 pos;
    
    Mine() {
        radius = 0;
        pos = Vector2();
    }
    
    Mine(float _radius, Vector2 _pos) {
        radius = _radius;
        pos = _pos;
    }
    
    static vector<Mine> parseMineStruct(const mxArray *structArray) { 
        int nelements = mxGetNumberOfElements(structArray);
        
        mexPrintf("Number of Mines: %d\n", nelements);
        
        vector<Mine> mineList = vector<Mine>();
        
        for (int i=0; i<nelements; i++) {
            mxArray *postmp = mxGetField(structArray, i, "pos");
            mxArray *radtmp = mxGetField(structArray, i, "radius");
            
            double radius = *mxGetPr(radtmp);
            Vector2 pos = Vector2(postmp);
            
            //mexPrintf("rad: %d\npos: %d, %d\n", (int)radius, (int)posx, (int)posy);
            mineList.push_back(Mine(radius, pos));
        }
        
        return mineList;
    }
};