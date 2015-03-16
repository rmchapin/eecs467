#include "Board.hpp"

std::string COLOR_NAMES[] = {"GREEN", "RED", "YELLOW"};

Board::Board()
{
}

Board::~Board()
{
}


void Board::boardInit(coord balls_[5], coord pos1, coord pos2, coord pos3, coord pos4, std::string color)
{
    // figure out x and y coords of each square
    double boardx2 = (pos1.x + pos4.x)/2;
    double boardx1 = (pos1.x + boardx2)/2;
    double boardx3 = (pos4.x + boardx2)/2;

    double boardy2 = (pos1.y + pos2.y)/2;
    double boardy1 = (pos2.y + boardy2)/2;
    double boardy3 = (pos1.y + boardy2)/2;

    coord min0 = {boardx1-15, boardy1-15};
    coord max0 = {boardx1+15, boardy1+15};
    board[0] = {YELLOW, min0, max0};

    coord min1 = {boardx1-15, boardy2-15};
    coord max1 = {boardx1+15, boardy2+15};
    board[1] = {YELLOW, min1, max1};

    coord min2 = {boardx1-15, boardy3-15};
    coord max2 = {boardx1+15, boardy3+15};
    board[2] = {YELLOW, min2, max2};

    coord min3 = {boardx2-15, boardy1-15};
    coord max3 = {boardx2+15, boardy1+15};
    board[3] = {YELLOW, min3, max3};

    coord min4 = {boardx2-15, boardy2-15};
    coord max4 = {boardx2+15, boardy2+15};
    board[4] = {YELLOW, min4, max4};

    coord min5 = {boardx2-15, boardy3-15};
    coord max5 = {boardx2+15, boardy3+15};
    board[5] = {YELLOW, min5, max5};

    coord min6 = {boardx3-15, boardy1-15};
    coord max6 = {boardx3+15, boardy1+15};
    board[6] = {YELLOW, min6, max6};

    coord min7 = {boardx3-15, boardy2-15};
    coord max7 = {boardx3+15, boardy2+15};
    board[7] = {YELLOW, min7, max7};

    coord min8 = {boardx3-15, boardy3-15};
    coord max8 = {boardx3+15, boardy3+15};
    board[8] = {YELLOW, min8, max8};

    if(color == "GREEN")
    {
        for(int i = 0; i < 5; i++)
        {
            freeBalls[i].position = balls_[i];
            freeBalls[i].color = GREEN;
        }
    }
    else // if color == "RED"
    {
        for(int i = 0; i < 5; i++)
        {
            freeBalls[i].position = balls_[i];
            freeBalls[i].color = RED;
        }
    }

    numFreeBalls = 5;
}

void Board::updateBoard(Ball *balls_, int size, std::string color)
{
    // for each ball, update board
    int j = 0; 
    for(int i = 0; i < size; i++)
    {
        //std::cout << "balls_[" << i << "]: (" << balls_[i].position.x << ", " << balls_[i].position.y << ")" << std::endl;
        int board_index = findBoardIndex(balls_[i]);
        if(board_index != -1)
        {
            board[board_index].ball = balls_[i].color;
        }
        else
        {
            //std::cout << "inside else, incrementing j" << std::endl;
            if(color == COLOR_NAMES[balls_[i].color])
            {
                freeBalls[j].position = balls_[i].position;
                freeBalls[j].color = balls_[i].color;
                j++;
            }
        }
    }
    numFreeBalls = j;
}

int Board::findBoardIndex(Ball ball)
{
    for(int i = 0; i < 9; i++)
    {
        if(ball.position.x > board[i].min_coord.x &&
           ball.position.x < board[i].max_coord.x &&
           ball.position.y > board[i].min_coord.y &&
           ball.position.y < board[i].max_coord.y)
        {
            return i;
        }
    }
    return -1;
}

Ball *Board::getFreeBalls()
{
    // note to self: RETURNING NULL IS USEFUL FOR DETERMINING WHEN THE GAME IS OVER
    if(numFreeBalls > 0)
    {
        return freeBalls;
    }
    else
    {
        return NULL;
    }
}

void Board::printInit()
{
    for(int i = 0; i < 9; i++)
    {
        std::cout << "(" << board[i].min_coord.x << ", " << board[i].min_coord.y << ") -> ("
                  << board[i].max_coord.x << ", " << board[i].max_coord.y << ")    ";
        if(i%3 == 2)
        {
            std::cout << std::endl;
        }
    }
}

void Board::print()
{
    // print board
    for(int i = 0, j = 0; i < 9; i++)
    {
        if(board[i].ball == GREEN) std::cout << "G ";
        else if(board[i].ball == RED) std::cout << "R ";
        else std::cout << "  ";
        if(i%3 != 2) std::cout << "| ";
        if(i%3 == 2){
            j++;
            std::cout << std::endl;
        }
        if(j < 3 && i%3 == 2) std::cout << "--+---+--" << std::endl;
    }

    // print free balls
    for(int i = 0; i < numFreeBalls; i++)
    {
        std::cout << "freeBalls[" << i << "]: (" << freeBalls[i].position.x << ", " << freeBalls[i].position.y << ")" << std::endl;
        std::cout << "                        " << COLOR_NAMES[freeBalls[i].color] << std::endl;
    }
}
