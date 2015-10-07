classdef node
    properties
        isWalkable
        nodePos
        gridX
        gridY
        
        gCost
        hCost
        parent
        heapIndex
    end
    
    methods
        function obj=node(pwalkable, ppos, pgridX, pgridY)
            obj.isWalkable = pwalkable;
            obj.nodePos = ppos;
            obj.gridX = pgridX;
            obj.gridY = pgridY;
        end
        
        function erg = fCost(obj)
            erg = obj.gCost + obj.hCost;
        end
        
        function erg=eq(a, b)
            erg = false;
            
            %if (a.gridX == b.gridX && a.gridY == b.gridY)
            %    erg = true;
            %end
        end
        
        function erg=compareTo(obj, item)
            if (item.fCost == obj.fCost)
                erg = obj.hCost - item.hCost;
            else
                erg = obj.fCost - item.fCost;
            end
            erg = -erg;
        end
    end
end