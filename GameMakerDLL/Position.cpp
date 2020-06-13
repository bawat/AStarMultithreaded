/*
 * Position.cpp
 *
 *  Created on: 18 Apr 2020
 *      Author: bawat
 */

#include <cmath>
#include "Position.h"

Position::Position(double x, double y):x{x},y{y} {}
double Position::distanceTo(Position other){
	return sqrt(pow(x - other.x, 2 ) + pow(y - other.y, 2));
}
bool Position::isClosestGridPositionTo(Position other, int gridSize){
	return abs(x - other.x) < gridSize/2.0 && abs(y - other.y) < gridSize/2.0;
}
bool Position::operator==(const Position &other) const{
	return x == other.x && y == other.y;
}
bool Position::operator!=(const Position &other) const{
	return !(*this == other);
}
Position Position::north(Position start, double distance){
	return Position{start.x, start.y - distance};
}
Position Position::northEast(Position start, double distance){
	return Position{start.x + distance, start.y - distance};
}
Position Position::east(Position start, double distance){
	return Position{start.x + distance, start.y};
}
Position Position::southEast(Position start, double distance){
	return Position{start.x + distance, start.y + distance};
}
Position Position::south(Position start, double distance){
	return Position{start.x, start.y + distance};
}
Position Position::southWest(Position start, double distance){
	return Position{start.x - distance, start.y + distance};
}
Position Position::west(Position start, double distance){
	return Position{start.x - distance, start.y};
}
Position Position::northWest(Position start, double distance){
	return Position{start.x - distance, start.y - distance};
}
