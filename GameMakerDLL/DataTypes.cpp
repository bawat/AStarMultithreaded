/*
 * DataTypes.h
 *
 *  Created on: 11 Apr 2020
 *      Author: bawat
 */

#ifndef DATATYPES_CPP_
#define DATATYPES_CPP_

#include <unordered_set>
#include <iostream>
#include <chrono>
#include <functional>
#include <vector>
#include <pthread.h>
#include "CollisionBox.cpp"
#include "Position.h"

using namespace std::chrono;

struct AStarAlgorithmParameters{
	Position start;
	Position end;
	int gridSize;
	pthread_t* assignedThread = new pthread_t;
	int requestID = -1;
	milliseconds creationTime;
	bool operator==(const AStarAlgorithmParameters &rhs){
		return start.x == rhs.start.x && start.y == rhs.start.y && end.x == rhs.end.x && end.y == rhs.end.y && gridSize == rhs.gridSize;
	}
	AStarAlgorithmParameters(double startX, double startY, double endX, double endY, int gridSize): start(findClearArea(startX, startY, gridSize)), end(findClearArea(endX, endY, gridSize)), gridSize(gridSize){

		creationTime = now();

		requestID = rand() % 100000 + 10;
		while(requestIDAlreadyUsed(requestID)){ requestID++; }

		previousInstances.push_back(this);

		clearExpiredInstances();
	};
	int getRequestID(){
		return requestID;
	}
private:
	Position findClearArea(double x, double y, int gridSize){

		if(!CollisionBox::pointCollision(x, y)) return Position{x, y};

		if(!CollisionBox::pointCollision(x + gridSize, y - gridSize)) return Position{x + gridSize, y - gridSize};
		if(!CollisionBox::pointCollision(x, y - gridSize)) return Position{x, y - gridSize};
		if(!CollisionBox::pointCollision(x - gridSize, y - gridSize)) return Position{x - gridSize, y - gridSize};

		if(!CollisionBox::pointCollision(x + gridSize, y)) return Position{x + gridSize, y};
		if(!CollisionBox::pointCollision(x - gridSize, y)) return Position{x - gridSize, y};

		if(!CollisionBox::pointCollision(x + gridSize, y + gridSize)) return Position{x + gridSize, y + gridSize};
		if(!CollisionBox::pointCollision(x, y + gridSize)) return Position{x, y + gridSize};
		if(!CollisionBox::pointCollision(x - gridSize, y + gridSize)) return Position{x - gridSize, y + gridSize};

		return Position{x, y};
	}
	inline static std::vector<AStarAlgorithmParameters*> previousInstances;
	static void clearExpiredInstances(){
		for(AStarAlgorithmParameters* instance : previousInstances){
			if(instance->hasExpired()){
				previousInstances.erase(std::remove(previousInstances.begin(), previousInstances.end(), instance), previousInstances.end());
				pthread_cancel(*instance->assignedThread);
				delete instance->assignedThread;
				delete instance;
			}
		}
	}
	static int requestIDAlreadyUsed(int requestID){
		for(AStarAlgorithmParameters* instance : previousInstances){
			if(instance->requestID == requestID) return true;
		}
		return false;
	}
	milliseconds now(){
		return duration_cast<milliseconds>(system_clock::now().time_since_epoch());
	}
	bool hasExpired(){
		return duration_cast<milliseconds>(now() - creationTime).count() > 3000;
	}
};

struct NodeProperties{
	Position pos{0,0};
	NodeProperties* origin = nullptr;
	double estimatedDistanceRemaining;
	double calculatedDistanceFromStartToMe = 0;
	double estimatedPathDistance;
	bool operator==(const NodeProperties &second) const {
		return pos == second.pos;
	}
	virtual ~NodeProperties(){};
protected:
	void estimateDistanceRemaining(Position goal){
		estimatedDistanceRemaining = pos.distanceTo(goal);
	}
	void generateDistanceFromStartToMe(){
		calculatedDistanceFromStartToMe = origin->calculatedDistanceFromStartToMe + origin->pos.distanceTo(pos);
	}
	//Rough distance estimate of how good this node is
	void generateAStarDistance(){
		estimatedPathDistance = calculatedDistanceFromStartToMe + estimatedDistanceRemaining;
	}
};
struct Node : public NodeProperties{
	Node(Position (*moveInDirection)(Position start, double distance), int gridSize, NodeProperties* origin, Position goal){
		this->pos = moveInDirection(origin->pos, gridSize);
		this->origin = origin;
		estimateDistanceRemaining(goal);
		generateDistanceFromStartToMe();
		generateAStarDistance();
	}
};

struct BeginningNode : public NodeProperties{
	BeginningNode(Position pos, Position goal){
		this->pos = pos;
		estimateDistanceRemaining(goal);
		generateAStarDistance();
	}
};

struct ClosestToEnd{
	bool operator()(NodeProperties* lhs,  NodeProperties* rhs)
	{
		return lhs->estimatedDistanceRemaining > rhs->estimatedDistanceRemaining;
	}
};

#endif /* DATATYPES_CPP_ */
