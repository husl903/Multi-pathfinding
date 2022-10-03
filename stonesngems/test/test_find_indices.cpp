#include <rnd/stonesngems.h>

#include <iostream>
#include <unordered_set>
#include <vector>

using namespace stonesngems;

void test_indices() {
    const std::string board_str = "14|14|-1|1|18|18|18|18|18|18|18|18|18|18|18|18|18|18|18|07|01|01|18|01|01|01|01|18|02|02|05|18|18|02|01|01|18|02|02|02|02|18|02|32|01|18|18|01|01|02|36|02|02|02|01|18|01|01|02|18|18|18|18|18|18|01|01|01|01|18|34|18|18|18|18|01|02|02|01|01|02|02|02|01|02|02|02|18|18|02|02|02|35|02|01|02|02|02|02|01|01|18|18|01|01|02|02|01|02|02|01|02|02|01|01|18|18|02|02|02|01|02|01|01|02|01|01|02|02|18|18|18|18|18|18|00|02|01|01|18|18|18|18|18|18|01|01|29|18|02|01|02|02|18|02|01|02|18|18|02|01|02|18|02|01|02|02|18|02|02|01|18|18|01|01|01|31|01|01|02|01|28|01|38|02|18|18|18|18|18|18|18|18|18|18|18|18|18|18|18";
    GameParameters params = kDefaultGameParams;
    params["game_board_str"] = GameParameter(board_str);
    RNDGameState state(params);

    // test 1
    {
        std::unordered_set<HiddenCellType> element_set = {HiddenCellType::kDiamond, HiddenCellType::kExitClosed, HiddenCellType::kExitOpen};
        std::vector<int> indices = state.get_indices_flat(element_set);
        std::cout << "Expected size: 2" << std::endl;
        std::cout << "Result: " << indices.size() << std::endl;
        for (auto const & idx : indices) {
            std::cout << idx << " ";
        }
        std::cout << std::endl;
    }

    // test 1
    {
        std::unordered_set<HiddenCellType> element_set = {
            HiddenCellType::kDiamond, HiddenCellType::kExitClosed, HiddenCellType::kExitOpen,
            HiddenCellType::kKeyRed, HiddenCellType::kGateRedClosed, HiddenCellType::kGateRedOpen,
            HiddenCellType::kKeyBlue, HiddenCellType::kGateBlueClosed, HiddenCellType::kGateBlueOpen,
            HiddenCellType::kKeyGreen, HiddenCellType::kGateGreenClosed, HiddenCellType::kGateGreenOpen,
            HiddenCellType::kKeyYellow, HiddenCellType::kGateYellowClosed, HiddenCellType::kGateYellowOpen, 
        };
        std::vector<int> indices = state.get_indices_flat(element_set);
        std::cout << "Expected size: 10" << std::endl;
        std::cout << "Result: " << indices.size() << std::endl;
        for (auto const & idx : indices) {
            std::cout << idx << " ";
        }
        std::cout << std::endl;
    }
}


int main() {
    test_indices();
}