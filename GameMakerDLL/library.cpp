#include <pthread.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <chrono>
#include "DSMap.cpp"
#include "DataTypes.cpp"
#include "CollisionBox.cpp"
#include "DijkstraPriorityQueue.cpp"

#define fn_export extern "C" __declspec (dllexport)
using namespace std;

//A special method name that is automatically called by Game Maker upon load
fn_export double RegisterCallbacks(char* arg1, char* arg2, char* arg3, char* arg4){
	DSMap::registerCallbacks(arg1, arg2, arg3, arg4);
	return 0;
}

fn_export double registerCollisionBox(double instanceID, double x1, double y1, double x2, double y2){
	CollisionBox::createNew((int)instanceID, x1, y1, x2, y2);
	return 0;
}

bool attemptAddToVisitList(NodeProperties* toAdd, DijkstraPriorityQueue& nodesToVisit,  std::unordered_set<NodeProperties, KeyHasher>& visitedNodes){
	if(CollisionBox::pointCollision(&toAdd->pos)) return false;

	//If we've already been here, we shouldn't process this node again
	if(visitedNodes.find(*toAdd) != visitedNodes.end()) return false;

	nodesToVisit.insertIfAbsent(toAdd);
	return true;
}

//TODO ADD CASE WHERE WE'RE STUCK AND CANT FIND ANYMORE NODES TO VISIT
void* calculateAStar(void* input){

	std::cout << "AStar Began running" << std::endl;

	AStarAlgorithmParameters desiredPath = *((struct AStarAlgorithmParameters*)input);

	DijkstraPriorityQueue 								nodesToVisit;
	std::unordered_set<NodeProperties, KeyHasher>		visitedNodes;
	std::vector<NodeProperties*> 						instancesToDelete;

	int gridSize = desiredPath.gridSize;
	NodeProperties* start = new BeginningNode{desiredPath.start, desiredPath.end};
	instancesToDelete.push_back(start);
	Position goal = desiredPath.end;
	NodeProperties* current = start;

	const auto nextDirections = {
			Position::north,
			Position::northEast,
			Position::east,
			Position::southEast,
			Position::southWest,
			Position::west,
			Position::northWest
	};

	int maxIterations = 1000;
	int iterations = 0;
	while(current->pos.distanceTo(desiredPath.end) > gridSize && iterations < maxIterations){

		for(auto nextDirection : nextDirections){
			Node* northernNode = new Node(nextDirection, gridSize, current, goal);
			attemptAddToVisitList(northernNode, nodesToVisit, visitedNodes);
			instancesToDelete.push_back(northernNode);
		}

		//Go to most promising position
		NodeProperties* foundNode = nodesToVisit.pop();
		visitedNodes.insert(*foundNode);
		current = foundNode;
		if(iterations++ % 100 == 0 && iterations > 50){
			std::cout << "Request " << desiredPath.requestID << " reached "  << iterations << " iterations. Start x" << desiredPath.start.x << " y" << desiredPath.start.y << ". End x" << desiredPath.end.x << " y" << desiredPath.end.y << std::endl;
		}
	}

	std::cout << "AStar Finished running this many iterations: " << iterations << std::endl;
	if(iterations == 0){
		std::cout << "Because CurrentPos was x:" << current->pos.x << " y:" << current->pos.y << std::endl;
		std::cout << "And DesiredEndPos was x:" << desiredPath.end.x << " y:" << desiredPath.end.y << std::endl;
		std::cout << "And DesiredBeginPos was x:" << desiredPath.start.x << " y:" << desiredPath.start.y << std::endl;
		std::cout << "And GridSize was " << gridSize << std::endl;
		std::cout << "And Loop Condition was " << !current->pos.isClosestGridPositionTo(desiredPath.end, gridSize) << std::endl;
	}

	/*
	while(current->pos != desiredPath.start){
		std::cout << "x:" << current->pos.x << " y:" << current->pos.y << endl;
		current = current->origin;
	}*/

	DSMap returnMap{};
	returnMap.addDouble("timedOut", iterations == maxIterations);
	int index = 0;
	for(; current->pos != desiredPath.start; index++){
		//std::cout << "x:" << current->pos.x << " y:" << current->pos.y << endl;
		returnMap
			.addDouble("position" + to_string(index) + "valueX", current->pos.x)
			.addDouble("position" + to_string(index) + "valueY", current->pos.y);
		current = current->origin;
	}
	returnMap.addDouble("pathWaypoints", index);
	returnMap.addDouble("requestID", desiredPath.getRequestID());
	returnMap.sendToGMS2();

	//Clear up the memory
	while(!instancesToDelete.empty()) {
		delete instancesToDelete.back();
		instancesToDelete.pop_back();
	}

	return nullptr;
}

pthread_t* startNewThread(AStarAlgorithmParameters* desiredPath){
	/* Create an extra thread which executes executeAsync */
	if(pthread_create(desiredPath->assignedThread, nullptr, calculateAStar, desiredPath)) {
		std::cout << "Error creating thread" << std::endl;
	}

	return desiredPath->assignedThread;
}

/*
 * Returns a random handle from 10 -> 100010;
 */
fn_export double asyncAStarDistance(double startX, double startY, double endX, double endY, double gridSize){
	AStarAlgorithmParameters* desiredPath = new AStarAlgorithmParameters(startX, startY, endX, endY, (int)gridSize);
	int handleID = desiredPath->getRequestID();

	pthread_detach(*startNewThread(desiredPath));

	return handleID;
}

int main(){

	registerCollisionBox(100259.00,  864.00,  64.00,  958.00,  111.00);
	registerCollisionBox(100257.00,  1376.00,  957.00,  1474.00,  1007.00);
	registerCollisionBox(100255.00,  64.00,  64.00,  158.00,  111.00);
	registerCollisionBox(100254.00,  672.00,  768.00,  766.00,  815.00);
	registerCollisionBox(100253.00,  1440.00,  64.00,  1534.00,  111.00);
	registerCollisionBox(100246.00,  960.00,  768.00,  1024.00,  832.00);
	registerCollisionBox(100245.00,  960.00,  832.00,  1024.00,  896.00);
	registerCollisionBox(100244.00,  1024.00,  832.00,  1088.00,  896.00);
	registerCollisionBox(100243.00,  1088.00,  832.00,  1152.00,  896.00);
	registerCollisionBox(100242.00,  1152.00,  832.00,  1216.00,  896.00);
	registerCollisionBox(100241.00,  1216.00,  832.00,  1280.00,  896.00);
	registerCollisionBox(100240.00,  1024.00,  768.00,  1088.00,  832.00);
	registerCollisionBox(100239.00,  1088.00,  768.00,  1152.00,  832.00);
	registerCollisionBox(100238.00,  1152.00,  768.00,  1216.00,  832.00);
	registerCollisionBox(100237.00,  1216.00,  768.00,  1280.00,  832.00);
	registerCollisionBox(100236.00,  1280.00,  832.00,  1344.00,  896.00);
	registerCollisionBox(100235.00,  1280.00,  768.00,  1344.00,  832.00);
	registerCollisionBox(100234.00,  1344.00,  832.00,  1408.00,  896.00);
	registerCollisionBox(100233.00,  1344.00,  768.00,  1408.00,  832.00);
	registerCollisionBox(100232.00,  1408.00,  832.00,  1472.00,  896.00);
	registerCollisionBox(100231.00,  1408.00,  768.00,  1472.00,  832.00);
	registerCollisionBox(100230.00,  1472.00,  832.00,  1536.00,  896.00);
	registerCollisionBox(100229.00,  1472.00,  768.00,  1536.00,  832.00);
	registerCollisionBox(100228.00,  1472.00,  704.00,  1536.00,  768.00);
	registerCollisionBox(100227.00,  1472.00,  640.00,  1536.00,  704.00);
	registerCollisionBox(100226.00,  1472.00,  576.00,  1536.00,  640.00);
	registerCollisionBox(100225.00,  1472.00,  512.00,  1536.00,  576.00);
	registerCollisionBox(100224.00,  1472.00,  448.00,  1536.00,  512.00);
	registerCollisionBox(100223.00,  896.00,  832.00,  960.00,  896.00);
	registerCollisionBox(100222.00,  832.00,  832.00,  896.00,  896.00);
	registerCollisionBox(100221.00,  896.00,  768.00,  960.00,  832.00);
	registerCollisionBox(100220.00,  832.00,  768.00,  896.00,  832.00);
	registerCollisionBox(100219.00,  832.00,  896.00,  896.00,  960.00);
	registerCollisionBox(100218.00,  896.00,  896.00,  960.00,  960.00);
	registerCollisionBox(100217.00,  960.00,  896.00,  1024.00,  960.00);
	registerCollisionBox(100216.00,  1024.00,  896.00,  1088.00,  960.00);
	registerCollisionBox(100215.00,  768.00,  768.00,  832.00,  832.00);
	registerCollisionBox(100214.00,  768.00,  832.00,  832.00,  896.00);
	registerCollisionBox(100213.00,  768.00,  896.00,  832.00,  960.00);
	registerCollisionBox(100212.00,  1088.00,  896.00,  1152.00,  960.00);
	registerCollisionBox(100211.00,  1152.00,  896.00,  1216.00,  960.00);
	registerCollisionBox(100210.00,  1216.00,  896.00,  1280.00,  960.00);
	registerCollisionBox(100209.00,  1280.00,  896.00,  1344.00,  960.00);
	registerCollisionBox(100208.00,  1344.00,  896.00,  1408.00,  960.00);
	registerCollisionBox(100207.00,  1408.00,  896.00,  1472.00,  960.00);
	registerCollisionBox(100206.00,  1760.00,  1056.00,  1824.00,  1120.00);
	registerCollisionBox(100205.00,  1792.00,  992.00,  1856.00,  1056.00);
	registerCollisionBox(100204.00,  1728.00,  992.00,  1792.00,  1056.00);
	registerCollisionBox(100203.00,  1760.00,  928.00,  1824.00,  992.00);
	registerCollisionBox(100202.00,  1536.00,  960.00,  1600.00,  1024.00);
	registerCollisionBox(100201.00,  1472.00,  960.00,  1536.00,  1024.00);
	registerCollisionBox(100200.00,  1472.00,  896.00,  1536.00,  960.00);
	registerCollisionBox(100199.00,  1536.00,  896.00,  1600.00,  960.00);
	registerCollisionBox(100198.00,  1536.00,  832.00,  1600.00,  896.00);
	registerCollisionBox(100197.00,  1536.00,  768.00,  1600.00,  832.00);
	registerCollisionBox(100196.00,  1664.00,  704.00,  1728.00,  768.00);
	registerCollisionBox(100195.00,  1664.00,  640.00,  1728.00,  704.00);
	registerCollisionBox(100194.00,  1792.00,  704.00,  1856.00,  768.00);
	registerCollisionBox(100193.00,  1856.00,  704.00,  1920.00,  768.00);
	registerCollisionBox(100192.00,  1856.00,  640.00,  1920.00,  704.00);
	registerCollisionBox(100191.00,  1792.00,  640.00,  1856.00,  704.00);
	registerCollisionBox(100190.00,  1856.00,  512.00,  1920.00,  576.00);
	registerCollisionBox(100189.00,  1856.00,  448.00,  1920.00,  512.00);
	registerCollisionBox(100188.00,  1664.00,  512.00,  1728.00,  576.00);
	registerCollisionBox(100187.00,  1728.00,  512.00,  1792.00,  576.00);
	registerCollisionBox(100186.00,  1728.00,  448.00,  1792.00,  512.00);
	registerCollisionBox(100185.00,  1664.00,  448.00,  1728.00,  512.00);
	registerCollisionBox(100184.00,  1344.00,  704.00,  1408.00,  768.00);
	registerCollisionBox(100183.00,  1280.00,  704.00,  1344.00,  768.00);
	registerCollisionBox(100182.00,  1216.00,  704.00,  1280.00,  768.00);
	registerCollisionBox(100181.00,  1152.00,  704.00,  1216.00,  768.00);
	registerCollisionBox(100180.00,  1088.00,  704.00,  1152.00,  768.00);
	registerCollisionBox(100179.00,  1024.00,  704.00,  1088.00,  768.00);
	registerCollisionBox(100178.00,  960.00,  64.00,  1024.00,  128.00);
	registerCollisionBox(100177.00,  960.00,  128.00,  1024.00,  192.00);
	registerCollisionBox(100176.00,  960.00,  192.00,  1024.00,  256.00);
	registerCollisionBox(100175.00,  960.00,  256.00,  1024.00,  320.00);
	registerCollisionBox(100174.00,  576.00,  384.00,  640.00,  448.00);
	registerCollisionBox(100173.00,  512.00,  384.00,  576.00,  448.00);
	registerCollisionBox(100172.00,  512.00,  320.00,  576.00,  384.00);
	registerCollisionBox(100171.00,  576.00,  320.00,  640.00,  384.00);
	registerCollisionBox(100170.00,  640.00,  320.00,  704.00,  384.00);
	registerCollisionBox(100169.00,  640.00,  384.00,  704.00,  448.00);
	registerCollisionBox(100168.00,  640.00,  448.00,  704.00,  512.00);
	registerCollisionBox(100167.00,  640.00,  512.00,  704.00,  576.00);
	registerCollisionBox(100166.00,  640.00,  576.00,  704.00,  640.00);
	registerCollisionBox(100165.00,  640.00,  640.00,  704.00,  704.00);
	registerCollisionBox(100164.00,  128.00,  704.00,  192.00,  768.00);
	registerCollisionBox(100163.00,  192.00,  704.00,  256.00,  768.00);
	registerCollisionBox(100162.00,  1536.00,  384.00,  1600.00,  448.00);
	registerCollisionBox(100161.00,  1536.00,  448.00,  1600.00,  512.00);
	registerCollisionBox(100160.00,  1536.00,  512.00,  1600.00,  576.00);
	registerCollisionBox(100159.00,  1536.00,  576.00,  1600.00,  640.00);
	registerCollisionBox(100158.00,  1536.00,  640.00,  1600.00,  704.00);
	registerCollisionBox(100157.00,  1536.00,  704.00,  1600.00,  768.00);
	registerCollisionBox(100156.00,  1408.00,  704.00,  1472.00,  768.00);
	registerCollisionBox(100155.00,  1408.00,  640.00,  1472.00,  704.00);
	registerCollisionBox(100154.00,  1408.00,  576.00,  1472.00,  640.00);
	registerCollisionBox(100153.00,  1408.00,  512.00,  1472.00,  576.00);
	registerCollisionBox(100152.00,  1408.00,  448.00,  1472.00,  512.00);
	registerCollisionBox(100151.00,  1408.00,  384.00,  1472.00,  448.00);
	registerCollisionBox(100150.00,  1472.00,  384.00,  1536.00,  448.00);
	registerCollisionBox(100149.00,  608.00,  128.00,  672.00,  192.00);
	registerCollisionBox(100148.00,  544.00,  128.00,  608.00,  192.00);
	registerCollisionBox(100147.00,  480.00,  128.00,  544.00,  192.00);
	registerCollisionBox(100146.00,  416.00,  128.00,  480.00,  192.00);
	registerCollisionBox(100145.00,  352.00,  128.00,  416.00,  192.00);
	registerCollisionBox(100144.00,  288.00,  128.00,  352.00,  192.00);
	registerCollisionBox(100143.00,  960.00,  320.00,  1024.00,  384.00);
	registerCollisionBox(100142.00,  960.00,  384.00,  1024.00,  448.00);
	registerCollisionBox(100141.00,  960.00,  448.00,  1024.00,  512.00);
	registerCollisionBox(100140.00,  960.00,  512.00,  1024.00,  576.00);
	registerCollisionBox(100139.00,  960.00,  576.00,  1024.00,  640.00);
	registerCollisionBox(100138.00,  960.00,  704.00,  1024.00,  768.00);
	registerCollisionBox(100137.00,  896.00,  704.00,  960.00,  768.00);
	registerCollisionBox(100136.00,  832.00,  704.00,  896.00,  768.00);
	registerCollisionBox(100135.00,  768.00,  704.00,  832.00,  768.00);
	registerCollisionBox(100134.00,  704.00,  704.00,  768.00,  768.00);
	registerCollisionBox(100133.00,  640.00,  704.00,  704.00,  768.00);
	registerCollisionBox(100132.00,  576.00,  704.00,  640.00,  768.00);
	registerCollisionBox(100131.00,  512.00,  704.00,  576.00,  768.00);
	registerCollisionBox(100130.00,  448.00,  704.00,  512.00,  768.00);
	registerCollisionBox(100129.00,  384.00,  704.00,  448.00,  768.00);
	registerCollisionBox(100128.00,  320.00,  704.00,  384.00,  768.00);
	registerCollisionBox(100127.00,  256.00,  704.00,  320.00,  768.00);
	registerCollisionBox(100126.00,  1024.00,  960.00,  1088.00,  1024.00);
	registerCollisionBox(100125.00,  1024.00,  1024.00,  1088.00,  1088.00);
	registerCollisionBox(100124.00,  1024.00,  1088.00,  1088.00,  1152.00);
	registerCollisionBox(100123.00,  1024.00,  1184.00,  1088.00,  1248.00);
	registerCollisionBox(100122.00,  1024.00,  1248.00,  1088.00,  1312.00);
	registerCollisionBox(100121.00,  960.00,  1248.00,  1024.00,  1312.00);
	registerCollisionBox(100120.00,  896.00,  1248.00,  960.00,  1312.00);
	registerCollisionBox(100119.00,  832.00,  1248.00,  896.00,  1312.00);
	registerCollisionBox(100118.00,  352.00,  960.00,  416.00,  1024.00);
	registerCollisionBox(100117.00,  416.00,  960.00,  480.00,  1024.00);
	registerCollisionBox(100116.00,  416.00,  1024.00,  480.00,  1088.00);
	registerCollisionBox(100115.00,  352.00,  1024.00,  416.00,  1088.00);
	registerCollisionBox(100114.00,  288.00,  1024.00,  352.00,  1088.00);
	registerCollisionBox(100113.00,  288.00,  960.00,  352.00,  1024.00);
	registerCollisionBox(100112.00,  960.00,  1472.00,  1024.00,  1536.00);
	registerCollisionBox(100111.00,  896.00,  1472.00,  960.00,  1536.00);
	registerCollisionBox(100110.00,  1984.00,  1408.00,  2048.00,  1472.00);
	registerCollisionBox(100109.00,  1984.00,  1344.00,  2048.00,  1408.00);
	registerCollisionBox(100108.00,  1984.00,  1280.00,  2048.00,  1344.00);
	registerCollisionBox(100107.00,  1984.00,  1216.00,  2048.00,  1280.00);
	registerCollisionBox(100106.00,  1984.00,  1152.00,  2048.00,  1216.00);
	registerCollisionBox(100105.00,  1984.00,  1088.00,  2048.00,  1152.00);
	registerCollisionBox(100104.00,  1984.00,  1024.00,  2048.00,  1088.00);
	registerCollisionBox(100103.00,  1984.00,  960.00,  2048.00,  1024.00);
	registerCollisionBox(100102.00,  1984.00,  896.00,  2048.00,  960.00);
	registerCollisionBox(100101.00,  1984.00,  832.00,  2048.00,  896.00);
	registerCollisionBox(100100.00,  1984.00,  768.00,  2048.00,  832.00);
	registerCollisionBox(100099.00,  1984.00,  704.00,  2048.00,  768.00);
	registerCollisionBox(100098.00,  1984.00,  640.00,  2048.00,  704.00);
	registerCollisionBox(100097.00,  1984.00,  576.00,  2048.00,  640.00);
	registerCollisionBox(100096.00,  1984.00,  512.00,  2048.00,  576.00);
	registerCollisionBox(100095.00,  1984.00,  448.00,  2048.00,  512.00);
	registerCollisionBox(100094.00,  1984.00,  384.00,  2048.00,  448.00);
	registerCollisionBox(100093.00,  1984.00,  320.00,  2048.00,  384.00);
	registerCollisionBox(100092.00,  1984.00,  256.00,  2048.00,  320.00);
	registerCollisionBox(100091.00,  1984.00,  192.00,  2048.00,  256.00);
	registerCollisionBox(100090.00,  1984.00,  128.00,  2048.00,  192.00);
	registerCollisionBox(100089.00,  1984.00,  64.00,  2048.00,  128.00);
	registerCollisionBox(100088.00,  1984.00,  1472.00,  2048.00,  1536.00);
	registerCollisionBox(100087.00,  1920.00,  1472.00,  1984.00,  1536.00);
	registerCollisionBox(100086.00,  1856.00,  1472.00,  1920.00,  1536.00);
	registerCollisionBox(100085.00,  1792.00,  1472.00,  1856.00,  1536.00);
	registerCollisionBox(100084.00,  1728.00,  1472.00,  1792.00,  1536.00);
	registerCollisionBox(100083.00,  1664.00,  1472.00,  1728.00,  1536.00);
	registerCollisionBox(100082.00,  1600.00,  1472.00,  1664.00,  1536.00);
	registerCollisionBox(100081.00,  1536.00,  1472.00,  1600.00,  1536.00);
	registerCollisionBox(100080.00,  1472.00,  1472.00,  1536.00,  1536.00);
	registerCollisionBox(100079.00,  1408.00,  1472.00,  1472.00,  1536.00);
	registerCollisionBox(100078.00,  1344.00,  1472.00,  1408.00,  1536.00);
	registerCollisionBox(100077.00,  1280.00,  1472.00,  1344.00,  1536.00);
	registerCollisionBox(100076.00,  1216.00,  1472.00,  1280.00,  1536.00);
	registerCollisionBox(100075.00,  1152.00,  1472.00,  1216.00,  1536.00);
	registerCollisionBox(100074.00,  1088.00,  1472.00,  1152.00,  1536.00);
	registerCollisionBox(100073.00,  1024.00,  1472.00,  1088.00,  1536.00);
	registerCollisionBox(100072.00,  1024.00,  1408.00,  1088.00,  1472.00);
	registerCollisionBox(100071.00,  960.00,  1408.00,  1024.00,  1472.00);
	registerCollisionBox(100070.00,  896.00,  1408.00,  960.00,  1472.00);
	registerCollisionBox(100069.00,  832.00,  1408.00,  896.00,  1472.00);
	registerCollisionBox(100068.00,  832.00,  1472.00,  896.00,  1536.00);
	registerCollisionBox(100067.00,  768.00,  1472.00,  832.00,  1536.00);
	registerCollisionBox(100066.00,  704.00,  1472.00,  768.00,  1536.00);
	registerCollisionBox(100065.00,  640.00,  1472.00,  704.00,  1536.00);
	registerCollisionBox(100064.00,  576.00,  1472.00,  640.00,  1536.00);
	registerCollisionBox(100063.00,  512.00,  1472.00,  576.00,  1536.00);
	registerCollisionBox(100062.00,  448.00,  1472.00,  512.00,  1536.00);
	registerCollisionBox(100061.00,  384.00,  1472.00,  448.00,  1536.00);
	registerCollisionBox(100060.00,  320.00,  1472.00,  384.00,  1536.00);
	registerCollisionBox(100059.00,  256.00,  1472.00,  320.00,  1536.00);
	registerCollisionBox(100058.00,  192.00,  1472.00,  256.00,  1536.00);
	registerCollisionBox(100057.00,  128.00,  1472.00,  192.00,  1536.00);
	registerCollisionBox(100056.00,  64.00,  1472.00,  128.00,  1536.00);
	registerCollisionBox(100055.00,  0.00,  1472.00,  64.00,  1536.00);
	registerCollisionBox(100054.00,  0.00,  1408.00,  64.00,  1472.00);
	registerCollisionBox(100053.00,  0.00,  1344.00,  64.00,  1408.00);
	registerCollisionBox(100052.00,  0.00,  1280.00,  64.00,  1344.00);
	registerCollisionBox(100051.00,  1984.00,  0.00,  2048.00,  64.00);
	registerCollisionBox(100050.00,  1920.00,  0.00,  1984.00,  64.00);
	registerCollisionBox(100049.00,  1856.00,  0.00,  1920.00,  64.00);
	registerCollisionBox(100048.00,  1792.00,  0.00,  1856.00,  64.00);
	registerCollisionBox(100047.00,  1728.00,  0.00,  1792.00,  64.00);
	registerCollisionBox(100046.00,  1664.00,  0.00,  1728.00,  64.00);
	registerCollisionBox(100045.00,  1600.00,  0.00,  1664.00,  64.00);
	registerCollisionBox(100044.00,  1536.00,  0.00,  1600.00,  64.00);
	registerCollisionBox(100043.00,  1472.00,  0.00,  1536.00,  64.00);
	registerCollisionBox(100042.00,  1408.00,  0.00,  1472.00,  64.00);
	registerCollisionBox(100041.00,  1344.00,  0.00,  1408.00,  64.00);
	registerCollisionBox(100040.00,  1280.00,  0.00,  1344.00,  64.00);
	registerCollisionBox(100039.00,  1216.00,  0.00,  1280.00,  64.00);
	registerCollisionBox(100038.00,  1152.00,  0.00,  1216.00,  64.00);
	registerCollisionBox(100037.00,  1088.00,  0.00,  1152.00,  64.00);
	registerCollisionBox(100036.00,  1024.00,  0.00,  1088.00,  64.00);
	registerCollisionBox(100035.00,  960.00,  0.00,  1024.00,  64.00);
	registerCollisionBox(100034.00,  896.00,  0.00,  960.00,  64.00);
	registerCollisionBox(100033.00,  832.00,  0.00,  896.00,  64.00);
	registerCollisionBox(100032.00,  768.00,  0.00,  832.00,  64.00);
	registerCollisionBox(100031.00,  704.00,  0.00,  768.00,  64.00);
	registerCollisionBox(100030.00,  640.00,  0.00,  704.00,  64.00);
	registerCollisionBox(100029.00,  576.00,  0.00,  640.00,  64.00);
	registerCollisionBox(100028.00,  512.00,  0.00,  576.00,  64.00);
	registerCollisionBox(100027.00,  448.00,  0.00,  512.00,  64.00);
	registerCollisionBox(100026.00,  384.00,  0.00,  448.00,  64.00);
	registerCollisionBox(100025.00,  320.00,  0.00,  384.00,  64.00);
	registerCollisionBox(100024.00,  256.00,  0.00,  320.00,  64.00);
	registerCollisionBox(100023.00,  192.00,  0.00,  256.00,  64.00);
	registerCollisionBox(100022.00,  128.00,  0.00,  192.00,  64.00);
	registerCollisionBox(100021.00,  64.00,  0.00,  128.00,  64.00);
	registerCollisionBox(100020.00,  0.00,  1216.00,  64.00,  1280.00);
	registerCollisionBox(100019.00,  0.00,  1152.00,  64.00,  1216.00);
	registerCollisionBox(100018.00,  0.00,  1088.00,  64.00,  1152.00);
	registerCollisionBox(100017.00,  0.00,  1024.00,  64.00,  1088.00);
	registerCollisionBox(100016.00,  0.00,  960.00,  64.00,  1024.00);
	registerCollisionBox(100015.00,  0.00,  896.00,  64.00,  960.00);
	registerCollisionBox(100014.00,  0.00,  832.00,  64.00,  896.00);
	registerCollisionBox(100013.00,  0.00,  768.00,  64.00,  832.00);
	registerCollisionBox(100012.00,  0.00,  704.00,  64.00,  768.00);
	registerCollisionBox(100011.00,  0.00,  640.00,  64.00,  704.00);
	registerCollisionBox(100010.00,  0.00,  576.00,  64.00,  640.00);
	registerCollisionBox(100009.00,  0.00,  512.00,  64.00,  576.00);
	registerCollisionBox(100008.00,  0.00,  448.00,  64.00,  512.00);
	registerCollisionBox(100007.00,  0.00,  384.00,  64.00,  448.00);
	registerCollisionBox(100006.00,  0.00,  320.00,  64.00,  384.00);
	registerCollisionBox(100005.00,  0.00,  256.00,  64.00,  320.00);
	registerCollisionBox(100004.00,  0.00,  192.00,  64.00,  256.00);
	registerCollisionBox(100003.00,  0.00,  128.00,  64.00,  192.00);
	registerCollisionBox(100002.00,  0.00,  64.00,  64.00,  128.00);
	registerCollisionBox(100001.00,  0.00,  0.00,  64.00,  64);

	asyncAStarDistance(544.00, 864.00, 729.09, 543.59, 32.00);

	int retVal = 20;
	pthread_exit(&retVal);//End the main method and return a value, but keep the process loaded until all threads have finished execution.

	return 0;
}
