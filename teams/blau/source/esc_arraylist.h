template<class T>
class Arraylist {
public:
    vector<T*> arrayItems;
    
    Arraylist () {
        arrayItems = vector<T*>();
    }
    
    void insert(T &item) {
        int insertIndex = arrayItems.size();
        arrayItems.push_back(&item);
        item.heapIndex = insertIndex;
    }
    
    bool contains(const T & item) {
        int index = item.heapIndex;
        if (index >= arrayItems.size()) {
            return false;
        }
        
        if (arrayItems[index] == &item) {
            return true;
        }
        
        return false;
    }
    
};