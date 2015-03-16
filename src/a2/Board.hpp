#ifndef _BOARD_HPP_
#define _BOARD_HPP_

#include "Arm.hpp"
#include <iostream>
#include <string>

enum Color{GREEN, RED, YELLOW};
extern std::string COLOR_NAMES[];

struct Ball
{
    Color color;
    coord position;
};

struct Square
{
    Color ball;
    coord min_coord;
    coord max_coord;
};

class Board
{
    private:
        Square board[9];
        Ball freeBalls[5];
        int numFreeBalls;
        int findBoardIndex(Ball ball);
    public:
        Board();
        ~Board();
        void boardInit(coord balls[5], coord pos1, coord pos2, coord pos3, coord pos4, std::string color);
        void updateBoard(Ball *balls, int size, std::string color);
        void printInit();
        void print();
        Ball *getFreeBalls();
};

#endif
