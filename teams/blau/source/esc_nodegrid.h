#pragma once

class Nodegrid {
public:
    const float constGridRadius = 0.003;
    const float constSafeBorder = 0.001;
    const float constSpaceballRadius = 0.01;
    const float constMineProxPenality = 0.0002;
    
    int gridSize = 0;
    vector<vector<Node>> nodeGrid;
    
    void setupNodeGrid(const vector<Mine> &mineList) {
        clock_t start;
        double duration;
        start = clock();
        
        gridSize = (int)(1/(constGridRadius*2));
        
        nodeGrid = vector<vector<Node>>();
        
        for (int x=0; x<=gridSize; x++) {
            nodeGrid.push_back(vector<Node>());
            for(int y=0; y<=gridSize; y++) {
                Vector2 gridPos = Vector2(x, y);
                Vector2 worldPos = gridPosToWorldPos(gridPos);
                bool is_walkable = isWalkableCheck(worldPos, mineList);
                Node node = Node(gridPos, worldPos, is_walkable);
                
                for (int i=0; i<mineList.size(); i++) {
                    float dist = (mineList[i].pos-worldPos).magnitude();
                    
                    node.mineCost += constMineProxPenality/dist/dist;
                }
                
                nodeGrid[x].push_back(node);
            }
        }
        
        duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
        //mexPrintf("Nodegrid (%u x %u nodes) calculated in %f s\n", nodeGrid.size(), nodeGrid[0].size(), (float)duration);
    }
    
    
    
    Vector2 gridPosToWorldPos(Vector2 gridPos) {
        return (gridPos*2*constGridRadius).clamp(0, 1);
    }
    
    Vector2 worldPosToGridPos(Vector2 worldPos) {
        Vector2 gridPos = worldPos/2/constGridRadius;
        gridPos.x = (int)gridPos.x;
        gridPos.y = (int)gridPos.y;
        
        return gridPos.clamp(0, gridSize);
    }
    
    Node &getNodeFromGridPos(Vector2 gridPos) {
        return nodeGrid[(int)gridPos.x][(int)gridPos.y];
    }
    
    Node &getValidNode(Vector2 gridPos) {
        Node &node1 = getNodeFromGridPos(gridPos);
        
        //mexPrintf("Pos x: %f, y: %f\n", node1.gridPos.x, node1.gridPos.y);
        
        //nothing to do
        if (node1.is_walkable)
            return node1;
        
        //check walkable neighbours
        for (int range = 1; range <= 5; range++) {
            vector<Node*> nbList = getNeighbours(node1, range);
            for(int i=0; i<nbList.size(); i++) {
                if (nbList[i]->is_walkable) {
                    return (*nbList[i]);
                }
            }
        }
        
        return node1;
    }
    
    
    const vector<Node*> getNeighbours(const Node &node, int range=1) {
        vector<Node*> neighbours = vector<Node*>();
        
        const Vector2 &gp = node.gridPos;
        for (int i = -range; i <= range; i++) {
            for (int j = -range; j <= range; j++) {
                if (i == 0 && j == 0)
                    continue;
                if (gp.x+i < 0 || gp.y+j < 0 || gp.x+i >= gridSize || gp.y+j >= gridSize)
                    continue;
                neighbours.push_back(&getNodeFromGridPos(Vector2(gp.x+i, gp.y+j)));
            }
        }
        
        return neighbours;
    }
    
    
    bool isWalkableCheck(Vector2 worldPos, const vector<Mine> &mineList) {
        float safeRadius = constSpaceballRadius + constSafeBorder;
        
        //check border
        if (worldPos.x < safeRadius || worldPos.x > 1-safeRadius || worldPos.y < safeRadius || worldPos.y > 1-safeRadius) {
            return false;
        }
        
        for (int i=0; i<mineList.size(); i++) {
            Vector2 dist = mineList[i].pos-worldPos;
            
            if (dist.magnitude()-mineList[i].radius <= safeRadius) {
                return false;
            }
        }
        return true;
    }
};