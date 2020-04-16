#include <queue>
#include <unordered_set>
#include <vector>

#include "DataTypes.cpp"

//https://stackoverflow.com/questions/17016175/c-unordered-map-using-a-custom-class-type-as-the-key
struct KeyHasher
{
  std::size_t operator()(const Position& k) const
  {

	        // Compute individual hash values and combine them using XOR
	        // and bit shifting:

	        return ((std::hash<int>()(k.x)
	                 ^ (std::hash<int>()(k.y) << 1)) >> 1);
  }
  std::size_t operator()(const NodeProperties& k) const
  {
	        return operator()(k.pos);
  }
};

using namespace std;
class DijkstraPriorityQueue{
	std::priority_queue<NodeProperties*, std::vector<NodeProperties*>, ClosestToEnd> nodesToVisit;
	std::unordered_set<Position, KeyHasher> nodesToVisitContentLookupTable;

	public:
		DijkstraPriorityQueue(){}
		void push(NodeProperties* add){
			nodesToVisit.push(add);
			nodesToVisitContentLookupTable.insert(add->pos);
		}
		NodeProperties* pop(){
			NodeProperties* smallestNode = nodesToVisit.top();
			nodesToVisit.pop();//Remove from priority queue
			nodesToVisitContentLookupTable.erase(smallestNode->pos);//Remove from content lookup table
			return smallestNode;
		}
		bool contains(NodeProperties* toCheck){
			return nodesToVisitContentLookupTable.contains(toCheck->pos);
		}
		void insertIfAbsent(NodeProperties* toInsert){
			if(!contains(toInsert)) push(toInsert);
		}
		bool empty(){
			return nodesToVisit.empty();
		}
		int size(){
			return nodesToVisit.size();
		}
};
