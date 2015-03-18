#ifndef _BOARD_HPP_
#define _BOARD_HPP_

#include "Arm.hpp"
#include "ImageProcessor.hpp"
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <vector>

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
        std::vector<Ball> freeBalls;
        int numFreeBalls;
        ImageProcessor *ip;
        Color playerColor;
        Color opponentColor;

        void clearBoard();
        int findBoardIndex(coord ball);
        void getCalibration(coord *data);
        bool withinBounds(int p1, int p2);
        int getCornersIndex(int x, int y, coord *calibrationData);
        void getInput(coord *calibrationData, coord *corners, std::vector<coord>& greenBalls, std::vector<coord>& redBalls, std::string filename);
        void getBalls(std::vector<coord>& greenBalls, std::vector<coord>& redBalls, std::string filename);
    public:
        Board(ImageProcessor *ip_, bool red);
        ~Board();
        void boardInit(std::string filename);
        void updateBoard(std::string filename);
        bool gameOver();
        void printInit();
        void print();
        std::vector<Ball> getFreeBalls();
        coord nextPick();
        inline int isCorner(int sq);
        inline int isCenter(int sq);
        int isWin(int sq);
        int blocksWin(int sq);
        int isFork(int sq);
        coord nextPlace();
};

#endif
