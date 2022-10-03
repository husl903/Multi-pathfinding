#include <cassert>
#include <sstream>
#include <string>
#include <vector>

#include "definitions.h"

namespace stonesngems {
namespace util {

Board parse_board_str(const std::string &board_str) {
    std::stringstream board_ss(board_str);
    std::string segment;
    std::vector<std::string> seglist;
    // string split on |
    while (std::getline(board_ss, segment, '|')) {
        seglist.push_back(segment);
    }

    assert(seglist.size() >= 2);

    // Get general info
    int rows = std::stoi(seglist[0]);
    int cols = std::stoi(seglist[1]);
    assert((int)seglist.size() == rows * cols + 4);
    int max_steps = std::stoi(seglist[2]);
    int max_gems = std::stoi(seglist[3]);
    Board board(rows, cols, static_cast<uint8_t>(max_gems), max_steps);

    // Parse grid
    for (std::size_t i = 4; i < seglist.size(); ++i) {
        HiddenCellType el = static_cast<HiddenCellType>(std::stoi(seglist[i]));
        board.item(i - 4) = static_cast<int8_t>(el);
        if (el == HiddenCellType::kAgent) {
            board.agent_pos = i - 4;
            board.agent_idx = i - 4;
        }
    }

    return board;
}

}    // namespace util
}    // namespace stonesngems
