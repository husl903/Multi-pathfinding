#include <rnd/stonesngems.h>

#include <iostream>

using namespace stonesngems;

void test_default() {
    GameParameters params = kDefaultGameParams;
    RNDGameState state(params);
    std::cout << state.is_solution() << std::endl;
}

int main() {
    test_default();
}