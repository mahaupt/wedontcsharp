classdef minheap
    properties
       items=node.empty; %creates node array
       currentItemCount=0;
    end
    
    methods
        function add(obj, item)
            item.heapIndex = obj.currentItemCount+1;
            obj.items{obj.currentItemCount+1} = item;
            obj.sortUp(item);
            obj.currentItemCount = obj.currentItemCount + 1;
        end
        
        function erg=removeFirst(obj)
            erg = obj.items{1};
            obj.currentItemCount = obj.currentItemCount -1;
            obj.items{1} = obj.items{obj.currentItemCount+1};
            obj.sortDown(items{0});
        end
        
        function updateItem(obj, item)
            obj.sortUp{item}
        end
        
        function erg=count(obj)
            erg = obj.currentItemCount;
        end
        
        function erg=contains(obj, item)
            ismember(obj.items, item);
        end
        
        function sortDown(obj, item)
            while(1)
                childIndexLeft = item.heapIndex*2+1;
                childIndexRight = item.heapIndex*2+2;
                swapIndex = 0;
                
                if (childIndexLeft < obj.currentItemCount)
                    swapIndex = childIndexLeft;

                    if (childIndexRight < obj.currentItemCount)
                        if (obj.items{childIndexLeft}.compareTo(obj.items{childIndexRight}) < 0)
                            swapIndex = childIndexRight;
                        end
                    end

                    if (item.CompareTo(obj.items{swapIndex}) < 0)
                        swap(item, obj.items{swapIndex});
                    
                    else
                        return
                    end
                else
                    return
                end
            end
        end
        
        function sortUp(obj, item)
            parentIndex = (item.heapIndex-1)/2;
            
            %return wenn Item ganz oben ist
            if (parentIndex < 1)
                return
            end
            
            while(1)
                parentItem = obj.items{parentIndex};
                if (item.compareTo(parentItem) > 0)
                    swap(item, parentItem);
                else
                    break;
                end

                parentIndex = (item.HeapIndex-1)/2;
            end
        end
        
        function swap(obj, itemA, itemB)
            obj.items{itemA.HeapIndex} = itemB;
            obj.items{itemB.HeapIndex} = itemA;

            itemAIndex = itemA.HeapIndex;
            itemA.HeapIndex = itemB.HeapIndex;
            itemB.HeapIndex = itemAIndex;
        end
    end
end