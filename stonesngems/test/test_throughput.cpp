#include <rnd/stonesngems.h>

#include <chrono>
#include <iostream>

using namespace stonesngems;

using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::milliseconds;

void test_throughput() {
    const std::string board_str = "14|14|-1|1|18|18|18|18|18|18|18|18|18|18|18|18|18|18|18|07|01|01|18|01|01|01|01|18|02|02|05|18|18|02|01|01|18|02|02|02|02|18|02|32|01|18|18|01|01|02|36|02|02|02|01|18|01|01|02|18|18|18|18|18|18|01|01|01|01|18|34|18|18|18|18|01|02|02|01|01|02|02|02|01|02|02|02|18|18|02|02|02|35|02|01|02|02|02|02|01|01|18|18|01|01|02|02|01|02|02|01|02|02|01|01|18|18|02|02|02|01|02|01|01|02|01|01|02|02|18|18|18|18|18|18|00|02|01|01|18|18|18|18|18|18|01|01|29|18|02|01|02|02|18|02|01|02|18|18|02|01|02|18|02|01|02|02|18|02|02|01|18|18|01|01|01|31|01|01|02|01|28|01|38|02|18|18|18|18|18|18|18|18|18|18|18|18|18|18|18";
    GameParameters params = kDefaultGameParams;
    params["game_board_str"] = GameParameter(board_str);
    RNDGameState state(params);
    int NUM_STEPS = 1000000;
    std::vector<RNDGameState> state_list;

    std::cout << "starting ..." << std::endl;

    auto t1 = high_resolution_clock::now();
    state_list.reserve(NUM_STEPS * 5);
    state_list.push_back(state);
    for (int i = 0; i < NUM_STEPS; ++i) {
        RNDGameState s = state_list[0];
        for (const auto dir : state_list[0].legal_actions()) {
            RNDGameState child = s;
            child.apply_action(dir);
            state_list.push_back(child);
        }
        std::vector<float> obs = state_list[0].get_observation();
        (void)obs;
        uint64_t hash = state_list[0].get_hash();
        (void)hash;
    }
    auto t2 = high_resolution_clock::now();
    duration<double, std::milli> ms_double = t2 - t1;

    std::cout << "Total time for " << NUM_STEPS << " steps: " << ms_double.count() / 1000 << std::endl;
    std::cout << "Time per step :  " << ms_double.count() / 1000 / NUM_STEPS << std::endl;
}

int main() {
    test_throughput();
}