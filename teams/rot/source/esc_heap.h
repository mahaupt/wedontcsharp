template<class T>
class Heap {
private:
    vector<T*> heapItems;
public:
    Heap() {
        heapItems = vector<T*>();
    }
    
    void print_r() {
        mexPrintf("Print_r Heap: \n");
        for(int i=0; i < heapItems.size(); i++) {
            mexPrintf("Node(%d) hi:%d: [%f, %f] fCost: %f, gCost: %f\n", i, heapItems[i]->heapIndex, heapItems[i]->gridPos.x, heapItems[i]->gridPos.y, heapItems[i]->fCost, heapItems[i]->gCost);
        }
    }
    
    void insert(T & item) {
        int heapIndex = heapItems.size();
        item.heapIndex = heapIndex;
        heapItems.push_back(&item);
        
        sortUp(item);
    }
    
    T &removeFirst() {
        T *firstItem = heapItems[0];
        int lastIndex = heapItems.size() - 1;
        heapItems[0] = heapItems[lastIndex];
        heapItems[0]->heapIndex = 0;
        heapItems.pop_back();
                
        sortDown(*heapItems[0]);
        
        return *firstItem;
    }
    
    void sortUp(T & item) {
        int parentIndex = (int) (item.heapIndex-1)/2;
        T & parentItem = *heapItems[parentIndex];

        if (item.heapIndex != parentIndex && parentItem.compareTo(item) > 0) {
            swap(item, parentItem);
            sortUp(item);
        }
    }
    
    void sortDown(T & item) {
        int childIndex1 = (int) (item.heapIndex*2+1);
        int childIndex2 = (int) (item.heapIndex*2+2);
        int lastIndex = heapItems.size()-1;
        
        //both indices out of range -> nothing to do
        if (childIndex1 > lastIndex && childIndex2 > lastIndex) {
            return;
        }
        
        //one index out of range
        if (childIndex2 > lastIndex && childIndex1 == lastIndex) {
            T &child1 = *heapItems[childIndex1];
            if (item.compareTo(child1) > 0) {
                swap(child1, item);
                return;
            }
        }
        
        //both indices in range
        T &child1 = *heapItems[childIndex1];
        T &child2 = *heapItems[childIndex2];
        
        if (item.compareTo(child1) > 0 && child1.compareTo(child2) < 0) {
            swap(child1, item);
            sortDown(item);
            return;
        }
        if (item.compareTo(child2) > 0) {
            swap(child2, item);
            sortDown(item);
            return;
        }
    }
    
    void swap(T & itemA, T & itemB) {
        int hpiA = itemA.heapIndex;
        int hpiB = itemB.heapIndex;
        
        heapItems[hpiA] = &itemB;
        heapItems[hpiB] = &itemA;
        
        itemB.heapIndex = hpiA;
        itemA.heapIndex = hpiB;
    }
    
    int size() {
        return heapItems.size();
    }
    
    bool contains(const T & item) {
        int index = item.heapIndex;
        if (index >= heapItems.size()) {
            return false;
        }
        
        if (heapItems[index] == &item) {
            return true;
        }
        
        return false;
    }
};

class HeapItem {
    int heapIndex;
};