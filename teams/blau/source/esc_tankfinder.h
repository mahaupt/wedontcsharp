class Tankfinder {
public:
    const float constSpielBes = 0.1;
    const float constNavSecurity = 0.02;
    Vector2 enemyPos;
    Vector2 enemyGes;
    vector<Mine> &mineList;
    
    Tankfinder(vector<Mine> &_mineList, Vector2 _enemyPos, Vector2 _enemyGes): mineList(_mineList) {
        enemyPos = _enemyPos;
        enemyGes = _enemyGes;
    }
    
    float findTanke(vector<Vector2> &tList, float pathPenalty, vector<Tanke*> &tankList, Vector2 prevPos, Vector2 prevPath) {
        int tankListSize = 0;
        for (int i = 0; i<tankList.size(); i++) {
            if (tankList[i] != 0) {
                tankListSize++;
            }
        }
        
        if (tankListSize <= 0) {
            tList.push_back(prevPos);
            return pathPenalty;
        }
        
        float penalty = 9999.0;
        
        for (int i = 0; i<tankList.size(); i++) {
            if (tankList[i] == 0)
                continue;
            
            float pen = calcTankPen(tankList[i]->pos, prevPos, prevPath);
            
            //remove tanke i from list
            Tanke* tmp = tankList[i];
            tankList[i] = 0;
            
            vector<Vector2> erg2 = vector<Vector2>();
            float erg1 = findTanke(erg2, pen + pathPenalty, tankList, tmp->pos, tmp->pos-prevPos);
            
            //add tanke i to list
            tankList[i] = tmp;
            
            if (erg1 < penalty) {
                penalty = erg1;
                tList = erg2;
            }
        }
        
        //add prev pos to tank list
        tList.push_back(prevPos);
        
        return penalty;
    }
    
    
    float calcTankPen(Vector2 tankPos, Vector2 prevPos, Vector2 prevPath) {
        float distPen = (tankPos - prevPos).magnitude();
        float dirPen  = getTimeToAlignVelocity((tankPos-prevPos).norm(), (prevPath).norm());
        float collPen = 0;
        float enemyPen = 0;
        if (corridorColliding(tankPos, prevPos, constNavSecurity)) {
            collPen = 1.5;
        }
        
        enemyPen = - ((enemyPos-tankPos).magnitude() + getTimeToAlignVelocity(enemyGes, (tankPos - enemyPos).norm()));
        
        //mexPrintf("dist: %f, dir: %f, coll: %f, enemy: %f\n", distPen, dirPen, collPen, enemyPen);
        return distPen + dirPen / 50 + collPen + enemyPen;
    }
    
    float getTimeToAlignVelocity(Vector2 vel1, Vector2 vec) {
        if(vel1.magnitude() <= 0.00001) {
            return 0;
        }
        
        float dotp = vel1.norm().dot(vec.norm());
        float angle = acos(dotp);
        if (dotp < 0) {
            angle = angle + 3.14159/2;
        }
        
        float deltaV = angle*vel1.magnitude();
        return deltaV/constSpielBes;
    }

    bool corridorColliding(Vector2 startp, Vector2 endp, float radius) {
        Vector2 dir = (endp-startp).norm();
        
        //middle line
        if (lineColliding(startp - dir*radius, endp + dir*radius, radius)) {
            return true;
        }
                
        return false;
    }
    
    bool lineColliding(Vector2 startp, Vector2 endp, float radius) {

        for (int i=1; i < mineList.size(); i++) {
            float dist = distanceLinePoint(startp, endp, mineList[i].pos);

            if (dist < mineList[i].radius+radius) {
                return true;
            }
        }
        return false;
    }
    
    float distanceLinePoint(Vector2 startp, Vector2 endp, Vector2 point) {
        float length = (startp - endp).magnitude();
        Vector2 dir = (endp-startp).norm();
        Vector2 n = dir.getPerpend();
       
        
        float y = (startp.y*n.x-point.y*n.x-n.y*startp.x+point.x*n.y)/(dir.x*n.y-dir.y*n.x);
        
        Vector2 linePoint = startp + dir*y;
        
        //point is outside of line
        if ((linePoint-startp).magnitude() > length || (linePoint-endp).magnitude() > length) {
            return 9999999;
        }
        
        return (point-linePoint).magnitude();
    }
};