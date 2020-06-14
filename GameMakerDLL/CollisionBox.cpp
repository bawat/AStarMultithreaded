#ifndef COLLISIONBOX_CPP_
#define COLLISIONBOX_CPP_

#include <vector>
#include <algorithm>
#include "Position.h"

using namespace std;
/*
 * Used to represent the map's state.
 * Handles collisions.
 */

class CollisionBox{
	private:
		inline static vector<CollisionBox> listOfInstances;
	private:
		int instanceID;
		double x1, y1, x2, y2;
		CollisionBox(int instanceID, double x1, double y1, double x2, double y2) :instanceID(instanceID), x1(x1), y1(y1), x2(x2), y2(y2){
			listOfInstances.push_back(*this);
		}
	public:
		static bool pointCollision(Position* pos){
			return pointCollision(pos->x, pos->y);
		}
		static bool pointCollision(double x, double y){
			for(CollisionBox instance : listOfInstances){
				if(instance.containsPoint(x, y)) return true;
			}

			return false;
		}
		static CollisionBox createNew(int instanceID, double x1, double y1, double x2, double y2){
			for(auto instance : listOfInstances){
				if(instance.instanceID == instanceID) return instance;
			}
			CollisionBox newBox{instanceID, x1, y1, x2, y2};
			return newBox;
		}
		static bool deleteByID(int instanceID){
			for(auto instance : listOfInstances){
				if(instance.instanceID == instanceID) {
					instance.remove();
					return true;
				}
			}
			return false;
		}
		bool containsPoint(double x, double y){
			return (x>=x1&&x<=x2) && (y>=y1&&y<=y2);
		}
		void remove(){
			listOfInstances.erase(std::remove(listOfInstances.begin(), listOfInstances.end(), *this), listOfInstances.end());
		}

		bool operator==(const CollisionBox &rightbox){
			return instanceID == rightbox.instanceID;
		}

		//Delete me
		static int debug_InstanceCount(){
			return listOfInstances.size();
		}
};


#endif /* COLLISIONBOX_CPP_ */
