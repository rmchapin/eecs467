#include "Board.hpp"
#include "ImageProcessor.hpp"
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

    ImageProcessor ip;
    ip.read_from_file("calibration_test2.txt");
    ip.calculate_x();

    Board board(&ip, true);
    board.boardInit("blob_output.txt");

    board.printInit();
    board.print();


    // green move
    board.updateBoard("blob_output1.txt");
    board.print();

    // red move
    board.updateBoard("blob_output2.txt");
    board.print();

    // green move
    board.updateBoard("blob_output3.txt");
    board.print();

    // red move
    board.updateBoard("blob_output4.txt");
    board.print();

    // green move
    board.updateBoard("blob_output5.txt");
    board.print();

    // red move
    board.updateBoard("blob_output6.txt");
    board.print();

    // green move
    board.updateBoard("blob_output7.txt");
    board.print();

    // red move
    board.updateBoard("blob_output8.txt");
    board.print();

    // red move
    board.updateBoard("blob_output9.txt");
    board.print();

    // red remove
    board.updateBoard("blob_output10.txt");
    board.print();

    return 0;
}
