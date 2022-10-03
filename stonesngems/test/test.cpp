#include <rnd/stonesngems.h>

#include <iostream>
#include <unordered_map>

using namespace stonesngems;
using namespace stonesngems::util;


const std::unordered_map<std::string, int> ActionMap{
    {"w", 1},
    {"d", 2},
    {"s", 3},
    {"a", 4},  
};


void test_play() {
    std::string board_str;
    std::cout << "Enter board str: ";
    std::cin >> board_str;
    GameParameters params = kDefaultGameParams;
    params["game_board_str"] = GameParameter(board_str);
    RNDGameState state(params);

    std::cout << state;
    std::cout << state.get_hash() << std::endl;
    
    std::string action_str;
    while (!state.is_terminal()) {
        std::cin >> action_str;
        state.apply_action(ActionMap.find(action_str) == ActionMap.end() ? 0 : ActionMap.at(action_str));
        std::cout << state;
        std::cout << state.get_hash() << std::endl;
        std::cout << std::endl;
    }
}


int main() {
    test_play();
}