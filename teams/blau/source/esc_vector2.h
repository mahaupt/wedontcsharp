class Vector2 {
public:
    float x;
    float y;
    
    Vector2(float _x, float _y) {
        x = _x;
        y = _y;
    }
    
    Vector2(const Vector2 &_vec) {
        x = _vec.x;
        y = _vec.y;
    }
    
    Vector2() {
        x = 0;
        y = 0;
    }
    
    Vector2(const mxArray *inpArray) {
        double *pos = mxGetPr(inpArray);
        x = (float)pos[0];
        y = (float)pos[1];
    }
    
    Vector2 operator* (int right) {
        return Vector2(x * right, y * right);
    }
    Vector2 operator* (float right) {
        return Vector2(x * right, y * right);
    }
    Vector2 operator/ (int right) {
        return Vector2(x / right, y / right);
    }
    Vector2 operator/ (float right) {
        return Vector2(x / right, y / right);
    }
    Vector2 operator- (Vector2 right) const {
        return Vector2(x-right.x, y-right.y);
    }
    Vector2 operator+ (Vector2 right) const {
        return Vector2(x+right.x, y+right.y);
    }
    
    
    Vector2 clamp(float min, float max) {
        float nx, ny;
        nx = x;
        ny = y;
        
        if (x < min)
            nx = min;
        if (x > max)
            nx = max;
        
        if (y < min)
            ny = min;
        if (y > max)
            ny = max;
        
        return Vector2(nx, ny);
    }
    
    
    float magnitude() {
        return sqrt(x*x+y*y);
    }
    Vector2 norm() {
        return Vector2(x,y)/magnitude();
    }
};