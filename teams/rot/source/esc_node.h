#pragma once

class Node : public HeapItem {
public:
    Vector2 gridPos;
    Vector2 worldPos;
    Node * parent;
    bool is_walkable;
    float hCost;
    float fCost;
    float gCost;
    float mineCost;
    int heapIndex;
    
    Node(Vector2 _gridPos, Vector2 _worldPos, bool _is_walkable) {
        gridPos = _gridPos;
        worldPos = _worldPos;
        is_walkable = _is_walkable;
        
        hCost = 0;
        fCost = 0;
        gCost = 0;
        mineCost = 0;
    }
    
    bool operator == (const Node &right) {
        if (gridPos.x == right.gridPos.x && gridPos.y == right.gridPos.y) {
            return true;
        }
        return false;
    }
    
    bool operator != (const Node &right) {
        return !operator==(right);
    }
    
    int compareTo(const Node & item) {
        if (fCost > item.fCost) {
            return 1;
        } else if (fCost < item.fCost) {
            return -1;
        } else if (hCost > item.hCost) {
            return 1;
        } else if (hCost < item.hCost) {
            return -1;
        }
        return 0;
    }
};