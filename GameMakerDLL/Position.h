/*
 * Position.h
 *
 *  Created on: 18 Apr 2020
 *      Author: bawat
 */

#ifndef POSITION_H_
#define POSITION_H_

struct Position{
	double x;
	double y;

	Position(double x, double y);
	double distanceTo(Position other);
	bool isClosestGridPositionTo(Position other, int gridSize);
	bool operator==(const Position &other) const;
	bool operator!=(const Position &other) const;
	static Position north(Position start, double distance);
	static Position northEast(Position start, double distance);
	static Position east(Position start, double distance);
	static Position southEast(Position start, double distance);
	static Position south(Position start, double distance);
	static Position southWest(Position start, double distance);
	static Position west(Position start, double distance);
	static Position northWest(Position start, double distance);
};

#endif /* POSITION_H_ */
