class Tanke {
public: 
    float radius;
    Vector2 pos;
    
    Tanke() {
        radius = 0;
        pos = Vector2();
    }
    
    Tanke(float _radius, Vector2 _pos) {
        radius = _radius;
        pos = _pos;
    }
    
    static vector<Tanke> parseTankeStruct(const mxArray *structArray) { 
        int nelements = mxGetNumberOfElements(structArray);
        
        //mexPrintf("Number of Tanken: %d\n", nelements);
        
        vector<Tanke> tankList = vector<Tanke>();
        
        for (int i=0; i<nelements; i++) {
            mxArray *postmp = mxGetField(structArray, i, "pos");
            mxArray *radtmp = mxGetField(structArray, i, "radius");
            
            double radius = *mxGetPr(radtmp);
            Vector2 pos = Vector2(postmp);
            
            //mexPrintf("rad: %d\npos: %d, %d\n", (int)radius, (int)posx, (int)posy);
            tankList.push_back(Tanke(radius, pos));
        }
        
        return tankList;
    }
};