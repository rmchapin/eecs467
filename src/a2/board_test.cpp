#include "Board.hpp"
#include <iostream>

using namespace std;

int main()
{
    coord initBalls[5];
    initBalls[0] = {0, 0};
    initBalls[1] = {1, 1};
    initBalls[2] = {2, 2};
    initBalls[3] = {3, 3};
    initBalls[4] = {4, 4};

    Board board;
    coord pos1 = {0, 40};
    coord pos2 = {0, -40};
    coord pos3 = {80, 40};
    coord pos4 = {80, -40};
    board.boardInit(initBalls, pos1, pos2, pos3, pos4, "GREEN");

    board.printInit();
    board.print();

    Ball b[5] = {{GREEN, {20, 0}}, {GREEN, {1, 1}}, {GREEN, {2, 2}}, {GREEN, {3, 3}}, {GREEN, {4, 4}}};

    // green move
    Ball nextballs1[5] = {b[0], b[1], b[2], b[3], b[4]};
    board.updateBoard(nextballs1, 5, "GREEN");
    board.print();

    // red move
    Ball nextballs2[6] = {{RED, {20, 20}}, b[1], b[2], b[3], b[4], b[0]};
    board.updateBoard(nextballs2, 6, "GREEN");
    board.print();

    // green move
    b[1].position.x = 40;
    b[1].position.y = 0;
    Ball nextballs3[6] = {{RED, {20, 20}}, b[1], b[2], b[3], b[4], b[0]};
    board.updateBoard(nextballs3, 6, "GREEN");
    board.print();

    // red move
    Ball nextballs4[7] = {{RED, {20, 20}}, b[2], b[3], {RED, {40, -20}}, b[4], b[1], b[0]};
    board.updateBoard(nextballs4, 7, "GREEN");
    board.print();

    // green move
    b[2].position.x = 60;
    b[2].position.y = 0;
    Ball nextballs5[7] = {{RED, {20, 20}}, b[0], b[1], {RED, {40, -20}}, b[2], b[3], b[4]};
    board.updateBoard(nextballs5, 7, "GREEN");
    board.print();

    // red move
    Ball nextballs6[8] = {{RED, {60, 20}}, {RED, {20, 20}}, b[0], b[1], {RED, {40, -20}}, b[2], b[3], b[4]};
    board.updateBoard(nextballs6, 8, "GREEN");
    board.print();

    // green move
    b[3].position.x = 60;
    b[3].position.y = -20;
    Ball nextballs7[8] = {{RED, {60, 20}}, {RED, {20, 20}}, b[0], b[1], b[2], {RED, {40, -20}}, b[3], b[4]};
    board.updateBoard(nextballs7, 8, "GREEN");
    board.print();

    // red move
    Ball nextballs8[9] = {{RED, {60, 20}}, {RED, {20, 20}}, b[0], b[1], {RED, {40, 20}}, b[2], b[3], {RED, {40, -20}}, b[4]};
    board.updateBoard(nextballs8, 9, "GREEN");
    board.print();

    // red move
    Ball nextballs9[10] = {{RED, {60, 20}}, {RED, {20, 20}}, b[0], {RED, {20, -20}}, b[1], {RED, {40, 20}}, b[2], b[3], b[4], {RED, {40, -20}}};
    board.updateBoard(nextballs9, 10, "GREEN");
    board.print();

    return 0;
}
