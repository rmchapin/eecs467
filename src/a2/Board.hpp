#ifndef _BOARD_HPP_
#define _BOARD_HPP_

#include "Arm.hpp"
#include "ImageProcessor.hpp"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>

enum Color{GREEN = 0, RED = 1, YELLOW = -1};
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
        ImageProcessor *ip;
        Color playerColor;

        void clearBoard();
        int findBoardIndex(coord ball);
        void getCalibration(coord *data);
        bool withinBounds(int p1, int p2);
        int getCornersIndex(int x, int y, coord *calibrationData);
        void getInput(coord *calibrationData, coord *corners, coord *greenBalls, coord *redBalls, std::string filename);
        void getBalls(coord *greenBalls, coord *redBalls, std::string filename);
    public:
        Board(ImageProcessor *ip_, bool red);
        ~Board();
        void boardInit(std::string filename);
        void updateBoard(std::string filename);
        bool gameOver();
        void printInit();
        void print();
        Ball *getFreeBalls();
        coord nextPick();
        inline int isCorner(int sq);
        inline int isCenter(int sq);
        int isWin(int sq);
        int blocksWin(int sq);
        coord nextPlace();
};

#endif
