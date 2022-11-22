#include "stonesngems_base.h"

#include <cstdint>

#include "definitions.h"

namespace stonesngems {

RNDGameState::RNDGameState(const GameParameters &params)
    : shared_state_ptr(std::make_shared<SharedStateInfo>(params)),
      board(util::parse_board_str(std::get<std::string>(params.at("game_board_str")))) {
    reset();
}

bool RNDGameState::operator==(const RNDGameState &other) const {
    return local_state == other.local_state && board == other.board;
}

// ---------------------------------------------------------------------------

void RNDGameState::reset() {
    // Board, local, and shared state info
    board = util::parse_board_str(shared_state_ptr->game_board_str);
    local_state = LocalState();
    local_state.steps_remaining = board.max_steps;
    shared_state_ptr->blob_chance = (board.cols * board.rows) * shared_state_ptr->blob_max_size;

    // zorbist hashing
    std::mt19937 gen(shared_state_ptr->rng_seed);
    std::uniform_int_distribution<uint64_t> dist(0);
    for (int channel = 0; channel < kNumHiddenCellType; ++channel) {
        for (int i = 0; i < board.cols * board.rows; ++i) {
            shared_state_ptr->zrbht[(channel * board.cols * board.rows) + i] = dist(gen);
        }
    }

    // Set initial hash
    for (int i = 0; i < board.cols * board.rows; ++i) {
        board.zorb_hash ^= shared_state_ptr->zrbht.at((board.item(i) * board.cols * board.rows) + i);
    }

    // In bounds fast access
    shared_state_ptr->in_bounds_board.clear();
    shared_state_ptr->in_bounds_board.insert(shared_state_ptr->in_bounds_board.end(),
                                             (board.cols + 2) * (board.rows + 2), true);
    // Pad the outer boarder
    for (int i = 0; i < board.cols + 2; ++i) {
        shared_state_ptr->in_bounds_board[i] = false;
        shared_state_ptr->in_bounds_board[(board.rows + 1) * (board.cols + 2) + i] = false;
    }
    for (int i = 0; i < board.rows + 2; ++i) {
        shared_state_ptr->in_bounds_board[i * (board.cols + 2)] = false;
        shared_state_ptr->in_bounds_board[i * (board.cols + 2) + board.cols + 1] = false;
    }
    // In bounds idx conversion table
    shared_state_ptr->board_to_inbounds.clear();
    for (int r = 0; r < board.rows; ++r) {
        for (int c = 0; c < board.cols; ++c) {
            shared_state_ptr->board_to_inbounds.push_back((board.cols + 2) * (r + 1) + c + 1);
        }
    }
}

void RNDGameState::apply_action(int action) {
    assert(action >= 0 && action < kNumActions);
    StartScan();

    int old_index = board.agent_idx;
    // Handle agent first
    UpdateAgent(board.agent_idx, static_cast<Directions>(action));
    if(action != 0 && old_index == board.agent_idx) return;
    // Handle all other items
    if(!board.is_opt_queue_event){
        for (int i = 0; i < board.rows * board.cols; ++i) {
            if (board.has_updated[i]) {    // Item already updated
                continue;
            }
            switch (board.item(i)) {
                // Handle non-compound types
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kStone):
                    UpdateStone(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kStoneFalling):
                    UpdateStoneFalling(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kDiamond):
                    UpdateDiamond(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kDiamondFalling):
                    UpdateDiamondFalling(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kNut):
                    UpdateNut(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kNutFalling):
                    UpdateNutFalling(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kBomb):
                    UpdateBomb(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kBombFalling):
                    UpdateBombFalling(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kExitClosed):
                    UpdateExit(i);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kBlob):
                    UpdateBlob(i);
                    break;
                default:
                    // Handle compound types
                    const Element &element = kCellTypeToElement[board.item(i) + 1];
                    if (IsButterfly(element)) {
                        UpdateButterfly(i, kButterflyToDirection.at(element));
                    } else if (IsFirefly(element)) {
                        UpdateFirefly(i, kFireflyToDirection.at(element));
                    } else if (IsOrange(element)) {
                        UpdateOrange(i, kOrangeToDirection.at(element));
                    } else if (IsMagicWall(element)) {
                        UpdateMagicWall(i);
                    } else if (IsExplosion(element)) {
                        UpdateExplosions(i);
                    }
                    break;
            }
        }
    }else{
        //  static std::vector<int> need_update_index_temp;
         for(int index_update = 0; index_update < board.need_update_index.size(); index_update++){
            int item_index = board.need_update_index[index_update];
            switch (board.item(item_index)) {
                // Handle non-compound types
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kStone):
                    UpdateStone(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kStoneFalling):
                    UpdateStoneFalling(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kDiamond):
                    UpdateDiamond(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kDiamondFalling):
                    UpdateDiamondFalling(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kNut):
                    UpdateNut(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kNutFalling):
                    UpdateNutFalling(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kBomb):
                    UpdateBomb(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kBombFalling):
                    UpdateBombFalling(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kExitClosed):
                    UpdateExit(item_index);
                    break;
                case static_cast<std::underlying_type_t<HiddenCellType>>(HiddenCellType::kBlob):
                    UpdateBlob(item_index);
                    break;
                default:
                    // Handle compound types
                    const Element &element = kCellTypeToElement[board.item(item_index) + 1];
                    if (IsButterfly(element)) {
                        UpdateButterfly(item_index, kButterflyToDirection.at(element));
                    } else if (IsFirefly(element)) {
                        UpdateFirefly(item_index, kFireflyToDirection.at(element));
                    } else if (IsOrange(element)) {
                        UpdateOrange(item_index, kOrangeToDirection.at(element));
                    } else if (IsMagicWall(element)) {
                        UpdateMagicWall(item_index);
                    } else if (IsExplosion(element)) {
                        UpdateExplosions(item_index);
                    }
                    break;
            }            
        }
        board.need_update_index.clear();
        board.need_update_index.swap(board.need_update_index_temp);
    }

    EndScan();
}

bool RNDGameState::is_terminal() const {
    // timeout or agent is either dead/in exit
    bool out_of_time = (board.max_steps > 0 && local_state.steps_remaining <= 0);
    return out_of_time || board.agent_pos < 0;
}

bool RNDGameState::is_solution() const {
    // not timeout and agent is in exit
    bool out_of_time = (board.max_steps > 0 && local_state.steps_remaining <= 0);
    return !out_of_time && board.agent_pos == kAgentPosExit;
}

std::vector<int> RNDGameState::legal_actions() const {
    return {Directions::kNoop, Directions::kUp, Directions::kRight, Directions::kDown, Directions::kLeft};
}

std::array<int, 3> RNDGameState::observation_shape() const {
    return {kNumVisibleCellType, board.cols, board.rows};
}

std::vector<float> RNDGameState::get_observation() const {
    int channel_length = board.cols * board.rows;
    std::vector<float> obs(kNumVisibleCellType * channel_length, 0);
    for (int i = 0; i < channel_length; ++i) {
        obs[static_cast<std::underlying_type_t<VisibleCellType>>(GetItem(i).visible_type) * channel_length + i] = 1;
    }
    return obs;
}

uint64_t RNDGameState::get_reward_signal() const {
    return local_state.reward_signal;
}

uint64_t RNDGameState::get_hash() const {
    return board.zorb_hash;
}

std::vector<std::pair<int, int>> RNDGameState::get_indices(HiddenCellType element) const {
    std::vector<std::pair<int, int>> indices;
    for (const auto &idx : board.find_all(static_cast<std::underlying_type_t<HiddenCellType>>(element))) {
        indices.push_back({idx / board.cols, idx % board.cols});
    }
    return indices;
}

std::vector<int> RNDGameState::get_indices_flat(std::unordered_set<HiddenCellType> element_set) const {
    std::vector<int> indices;
    for (int i = 0; i < (int)board.grid.size(); ++i) {
        if (element_set.find(static_cast<HiddenCellType>(board.grid[i])) != element_set.end()) {
            indices.push_back(i);
        }
    }
    return indices;
}

std::unordered_set<RewardCodes> RNDGameState::get_valid_rewards() const {
    std::unordered_set<RewardCodes> reward_codes;
    for (int i = 0; i < (int)board.grid.size(); ++i) {
        HiddenCellType el = static_cast<HiddenCellType>(board.grid[i]);
        if (kElementToRewardMap.find(el) != kElementToRewardMap.end()) {
            reward_codes.insert(kElementToRewardMap.at(el));
        }
    }
    return reward_codes;
}

int RNDGameState::get_agent_pos() const {
    return board.agent_pos;
}

int RNDGameState::get_agent_index() const {
    return board.agent_idx;
}

int8_t RNDGameState::get_index_item(int index) const {
    return board.item(index);
}

std::ostream &operator<<(std::ostream &os, const RNDGameState &state) {
    for (int h = 0; h < state.board.rows; ++h) {
        for (int w = 0; w < state.board.cols; ++w) {
            os << kCellTypeToElement[state.board.grid[h * state.board.cols + w] + 1].id;
        }
        os << std::endl;
    }
    return os;
}

// ---------------------------------------------------------------------------

// Not safe, assumes InBounds has been called (or used in conjunction)
int RNDGameState::IndexFromAction(int index, int action) const {
    switch (action) {
        case Directions::kNoop:
            return index;
        case Directions::kUp:
            return index - board.cols;
        case Directions::kRight:
            return index + 1;
        case Directions::kDown:
            return index + board.cols;
        case Directions::kLeft:
            return index - 1;
        case Directions::kUpRight:
            return index - board.cols + 1;
        case Directions::kDownRight:
            return index + board.cols + 1;
        case Directions::kUpLeft:
            return index - board.cols - 1;
        case Directions::kDownLeft:
            return index + board.cols - 1;
        default:
            __builtin_unreachable();
    }
}

bool RNDGameState::InBounds(int index, int action) const {
    return shared_state_ptr->in_bounds_board[shared_state_ptr->board_to_inbounds[IndexFromAction(index, action)]];
}

bool RNDGameState::IsType(int index, const Element &element, int action) const {
    int new_index = IndexFromAction(index, action);
    return InBounds(index, action) && GetItem(new_index) == element;
}

bool RNDGameState::HasProperty(int index, int property, int action) const {
    int new_index = IndexFromAction(index, action);
    return InBounds(index, action) && ((GetItem(new_index).properties & property) > 0);
}

void RNDGameState::MoveItem(int index, int action) {
    int new_index = IndexFromAction(index, action);
    if(curr_gem_index == index) curr_gem_index = new_index;
    board.zorb_hash ^= shared_state_ptr->zrbht.at((board.item(new_index) * board.cols * board.rows) + new_index);
    board.item(new_index) = board.item(index);
    board.zorb_hash ^= shared_state_ptr->zrbht.at((board.item(new_index) * board.cols * board.rows) + new_index);
    // grid_.ids[new_index] = grid_.ids[index];

    board.zorb_hash ^= shared_state_ptr->zrbht.at((board.item(index) * board.cols * board.rows) + index);
    board.item(index) = ElementToItem(kElEmpty);
    board.zorb_hash ^= shared_state_ptr->zrbht.at((ElementToItem(kElEmpty) * board.cols * board.rows) + index);
    board.has_updated[new_index] = true;
    // grid_.ids[index] = ++id_counter_;
}

void RNDGameState::SetItem(int index, const Element &element, int id, int action) {
    (void)id;
    int new_index = IndexFromAction(index, action);
    board.zorb_hash ^= shared_state_ptr->zrbht.at((board.item(new_index) * board.cols * board.rows) + new_index);
    board.item(new_index) = ElementToItem(element);
    board.zorb_hash ^= shared_state_ptr->zrbht.at((ElementToItem(element) * board.cols * board.rows) + new_index);
    // grid_.ids[new_index] = id;
    board.has_updated[new_index] = true;
}

const Element &RNDGameState::GetItem(int index, int action) const {
    int new_index = IndexFromAction(index, action);
    return kCellTypeToElement[board.item(new_index) + 1];
}

bool RNDGameState::IsTypeAdjacent(int index, const Element &element) const {
    return IsType(index, element, Directions::kUp) || IsType(index, element, Directions::kLeft) ||
           IsType(index, element, Directions::kDown) || IsType(index, element, Directions::kRight);
}

// ---------------------------------------------------------------------------

bool RNDGameState::CanRollLeft(int index) const {
    return HasProperty(index, ElementProperties::kRounded, Directions::kDown) &&
           IsType(index, kElEmpty, Directions::kLeft) && IsType(index, kElEmpty, Directions::kDownLeft);
}

bool RNDGameState::CanRollRight(int index) const {
    return HasProperty(index, ElementProperties::kRounded, Directions::kDown) &&
           IsType(index, kElEmpty, Directions::kRight) && IsType(index, kElEmpty, Directions::kDownRight);
}

void RNDGameState::RollLeft(int index, const Element &element) {
    SetItem(index, element, -1);
    MoveItem(index, Directions::kLeft);
}

void RNDGameState::RollRight(int index, const Element &element) {
    SetItem(index, element, -1);
    MoveItem(index, Directions::kRight);
}

void RNDGameState::Push(int index, const Element &stationary, const Element &falling, int action) {
    int new_index = IndexFromAction(index, action);
    // Check if same direction past element is empty so that theres room to push
    if (IsType(new_index, kElEmpty, action)) {

        // Check if the element will become stationary or falling
        int next_index = IndexFromAction(new_index, action);
        bool is_empty = IsType(next_index, kElEmpty, Directions::kDown);
        SetItem(new_index, is_empty ? falling : stationary, -1, action);
        // Move the agent
        MoveItem(index, action);
        board.agent_pos = IndexFromAction(index, action);    // Assume only agent is pushing?
        board.agent_idx = IndexFromAction(index, action);    // Assume only agent is pushing?
        if(board.is_opt_queue_event){
            board.need_update_index_temp.push_back(next_index);
        }        
    }
}

void RNDGameState::MoveThroughMagic(int index, const Element &element) {
    // Check if magic wall is still active
    if (local_state.magic_wall_steps <= 0) {
        return;
    }
    local_state.magic_active = true;
    int index_below = IndexFromAction(index, Directions::kDown);
    // Need to ensure cell below magic wall is empty (so item can pass through)
    if (IsType(index_below, kElEmpty, Directions::kDown)) {
        SetItem(index, kElEmpty, -1);
        SetItem(index_below, element, -1, Directions::kDown);
    }
}

void RNDGameState::Explode(int index, const Element &element, int action) {
    int new_index = IndexFromAction(index, action);
    auto it = kElementToExplosion.find(GetItem(new_index));
    const Element &ex = (it == kElementToExplosion.end()) ? kElExplosionEmpty : it->second;
    if (GetItem(new_index) == kElAgent) {
        board.agent_pos = kAgentPosDie;
    }
    SetItem(new_index, element, -1);
    // Recursively check all directions for chain explosions
    for (int dir = 0; dir < kNumDirections; ++dir) {
        if (dir == Directions::kNoop || !InBounds(new_index, dir)) {
            continue;
        }
        if (HasProperty(new_index, ElementProperties::kCanExplode, dir)) {
            Explode(new_index, ex, dir);
        } else if (HasProperty(new_index, ElementProperties::kConsumable, dir)) {
            SetItem(new_index, ex, -1, dir);
            if (GetItem(new_index, dir) == kElAgent) {
                board.agent_pos = kAgentPosDie;
            }
        }
    }
}

// ---------------------------------------------------------------------------

void RNDGameState::UpdateStone(int index) {
    // If no gravity, do nothing
    if (!shared_state_ptr->gravity) {
        return;
    }
    // std::cout << "stone " << index  << std::endl;
    // Boulder falls if empty below
    if (IsType(index, kElEmpty, Directions::kDown)) {
        SetItem(index, kElStoneFalling, -1);
        UpdateStoneFalling(index);
    } else if (CanRollLeft(index)) {    // Roll left/right if possible
        RollLeft(index, kElStoneFalling);
        // std::cout << "roll left " << index  << std::endl;
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index - 1);
    } else if (CanRollRight(index)) {
        RollRight(index, kElStoneFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + 1);
    } else{
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    }
    
}

void RNDGameState::UpdateStoneFalling(int index) {
    // Continue to fall as normal
    if (IsType(index, kElEmpty, Directions::kDown)) {
        MoveItem(index, Directions::kDown);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + board.cols);        
    } else if (HasProperty(index, ElementProperties::kCanExplode, Directions::kDown)) {
        // Falling stones can cause elements to explode
        auto it = kElementToExplosion.find(GetItem(index, Directions::kDown));
        Explode(index, (it == kElementToExplosion.end()) ? kElExplosionEmpty : it->second, Directions::kDown);
    } else if (IsType(index, kElWallMagicOn, Directions::kDown) ||
               IsType(index, kElWallMagicDormant, Directions::kDown)) {
        MoveThroughMagic(index, kMagicWallConversion.at(GetItem(index)));
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + board.cols);
    } else if (IsType(index, kElNut, Directions::kDown)) {
        // Falling on a nut, crack it open to reveal a diamond!
        SetItem(index, kElDiamond, -1, Directions::kDown);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + board.cols);
    } else if (IsType(index, kElNut, Directions::kDown)) {
        // Falling on a bomb, explode!
        auto it = kElementToExplosion.find(GetItem(index));
        Explode(index, (it == kElementToExplosion.end()) ? kElExplosionEmpty : it->second);
    } else if (CanRollLeft(index)) {    // Roll left/right
        RollLeft(index, kElStoneFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index - 1);
    } else if (CanRollRight(index)) {
        RollRight(index, kElStoneFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + 1);
    } else {
        // Default options is for falling stones to become stationary
        SetItem(index, kElStone, -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);        
    }
}

void RNDGameState::UpdateDiamond(int index) {
    // If no gravity, do nothing
    if (!shared_state_ptr->gravity) {
        return;
    }

    // Diamond falls if empty below
    if (IsType(index, kElEmpty, Directions::kDown)) {
        SetItem(index, kElDiamondFalling, -1);
        UpdateDiamondFalling(index);
    } else if (CanRollLeft(index)) {    // Roll left/right if possible
        RollLeft(index, kElDiamondFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index - 1);
    } else if (CanRollRight(index)) {
        RollRight(index, kElDiamondFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + 1);
    } else {
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    }
}

void RNDGameState::UpdateDiamondFalling(int index) {
    // Continue to fall as normal
    if (IsType(index, kElEmpty, Directions::kDown)) {
        MoveItem(index, Directions::kDown);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + board.cols);        
    } else if (HasProperty(index, ElementProperties::kCanExplode, Directions::kDown) &&
               !IsType(index, kElBomb, Directions::kDown) && !IsType(index, kElBombFalling, Directions::kDown)) {
        // Falling diamonds can cause elements to explode (but not bombs)
        auto it = kElementToExplosion.find(GetItem(index, Directions::kDown));
        Explode(index, (it == kElementToExplosion.end()) ? kElExplosionEmpty : it->second, Directions::kDown);
    } else if (IsType(index, kElWallMagicOn, Directions::kDown) ||
               IsType(index, kElWallMagicDormant, Directions::kDown)) {
        MoveThroughMagic(index, kMagicWallConversion.at(GetItem(index)));
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + board.cols);
    } else if (CanRollLeft(index)) {    // Roll left/right
        RollLeft(index, kElDiamondFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index - 1);
    } else if (CanRollRight(index)) {
        RollRight(index, kElDiamondFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + 1);
    } else {
        // Default options is for falling diamond to become stationary
        SetItem(index, kElDiamond, -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    }
}

void RNDGameState::UpdateNut(int index) {
    // If no gravity, do nothing
    if (!shared_state_ptr->gravity) {
        return;
    }

    // Nut falls if empty below
    if (IsType(index, kElEmpty, Directions::kDown)) {
        SetItem(index, kElNutFalling, -1);
        UpdateNutFalling(index);
    } else if (CanRollLeft(index)) {    // Roll left/right
        RollLeft(index, kElNutFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index - 1);
    } else if (CanRollRight(index)) {
        RollRight(index, kElNutFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + 1);
    }
}

void RNDGameState::UpdateNutFalling(int index) {
    // Continue to fall as normal
    if (IsType(index, kElEmpty, Directions::kDown)) {
        MoveItem(index, Directions::kDown);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + board.cols);
    } else if (CanRollLeft(index)) {    // Roll left/right
        RollLeft(index, kElNutFalling);
    } else if (CanRollRight(index)) {
        RollRight(index, kElNutFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + 1);
    } else {
        // Default options is for falling nut to become stationary
        SetItem(index, kElNut, -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    }
}

void RNDGameState::UpdateBomb(int index) {
    // If no gravity, do nothing
    if (!shared_state_ptr->gravity) {
        return;
    }

    // Bomb falls if empty below
    if (IsType(index, kElEmpty, Directions::kDown)) {
        SetItem(index, kElBombFalling, -1);
        UpdateBombFalling(index);
    } else if (CanRollLeft(index)) {    // Roll left/right
        RollLeft(index, kElBomb);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index - 1);
    } else if (CanRollRight(index)) {
        RollRight(index, kElBomb);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + 1);
    }
}

void RNDGameState::UpdateBombFalling(int index) {
    // Continue to fall as normal
    if (IsType(index, kElEmpty, Directions::kDown)) {
        MoveItem(index, Directions::kDown);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + board.cols);
    } else if (CanRollLeft(index)) {    // Roll left/right
        RollLeft(index, kElBombFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index - 1);
    } else if (CanRollRight(index)) {
        RollRight(index, kElBombFalling);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index + 1);
    } else {
        // Default options is for bomb to explode if stopped falling
        auto it = kElementToExplosion.find(GetItem(index));
        Explode(index, (it == kElementToExplosion.end()) ? kElExplosionEmpty : it->second);
    }
}

void RNDGameState::UpdateExit(int index) {
    // Open exit if enough gems collected
    if (local_state.gems_collected >= board.gems_required) {
        SetItem(index, kElExitOpen, -1);
    }else{
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    }
}

void RNDGameState::UpdateAgent(int index, int action) {
    // If action results not in bounds, don't do anything
    if (!InBounds(index, action)) {
        return;
    }

    if (IsType(index, kElEmpty, action) || IsType(index, kElDirt, action)) {    // Move if empty/dirt
        MoveItem(index, action);
        board.agent_pos = IndexFromAction(index, action);
        board.agent_idx = IndexFromAction(index, action);
    } else if (IsType(index, kElDiamond, action) || IsType(index, kElDiamondFalling, action)) {    // Collect gems
        ++local_state.gems_collected;
        local_state.current_reward += kPointMap.at(GetItem(index, action).cell_type);
        local_state.reward_signal |= RewardCodes::kRewardCollectDiamond;
        MoveItem(index, action);
        board.agent_pos = IndexFromAction(index, action);
        board.agent_idx = IndexFromAction(index, action);
    } else if (IsActionHorz(action) && HasProperty(index, ElementProperties::kPushable, action)) {
        // Push stone, nut, or bomb if action is horizontal
        Push(index, GetItem(index, action), kElToFalling.at(GetItem(index, action)), action);
    } else if (IsKey(GetItem(index, action))) {
        // Collecting key, set gate open
        Element key_type = GetItem(index, action);
        OpenGate(kKeyToGate.at(key_type));
        MoveItem(index, action);
        board.agent_pos = IndexFromAction(index, action);
        board.agent_idx = IndexFromAction(index, action);
        local_state.reward_signal |= RewardCodes::kRewardCollectKey;
        local_state.reward_signal |= kKeyToSignal.at(key_type);
    } else if (IsOpenGate(GetItem(index, action))) {
        // Walking through an open gate, with traversable element on other side
        int index_gate = IndexFromAction(index, action);
        if (HasProperty(index_gate, ElementProperties::kTraversable, action)) {
            // Correct for landing on traversable elements
            if (IsType(index_gate, kElDiamond, action) || IsType(index_gate, kElDiamondFalling, action)) {
                ++local_state.gems_collected;
                local_state.current_reward += kPointMap.at(GetItem(index_gate, action).cell_type);
                local_state.reward_signal |= RewardCodes::kRewardCollectDiamond;
            } else if (IsKey(GetItem(index_gate, action))) {
                Element key_type = GetItem(index_gate, action);
                OpenGate(kKeyToGate.at(key_type));
                local_state.reward_signal |= RewardCodes::kRewardCollectKey;
                local_state.reward_signal |= kKeyToSignal.at(key_type);
            }
            // Move agent through gate
            SetItem(index_gate, kElAgent, -1, action);
            SetItem(index, kElEmpty, -1);
            board.agent_pos = IndexFromAction(index_gate, action);
            board.agent_idx = IndexFromAction(index_gate, action);
            local_state.reward_signal |= RewardCodes::kRewardWalkThroughGate;
            local_state.reward_signal |= kGateToSignal.at(GetItem(index_gate));
        }
    } else if (IsType(index, kElExitOpen, action)) {
        // Walking into exit after collecting enough gems
        MoveItem(index, action);
        SetItem(index, kElAgentInExit, -1, action);
        board.agent_pos = kAgentPosExit;
        board.agent_idx = IndexFromAction(index, action);
        local_state.reward_signal |= RewardCodes::kRewardWalkThroughExit;
        local_state.current_reward += local_state.steps_remaining * 100 / board.max_steps;
    }
}

void RNDGameState::UpdateFirefly(int index, int action) {
    int new_dir = kRotateLeft[action];
    if (IsTypeAdjacent(index, kElAgent) || IsTypeAdjacent(index, kElBlob)) {
        // Explode if touching the agent/blob
        auto it = kElementToExplosion.find(GetItem(index));
        Explode(index, (it == kElementToExplosion.end()) ? kElExplosionEmpty : it->second);
    } else if (IsType(index, kElEmpty, new_dir)) {
        // Fireflies always try to rotate left, otherwise continue forward
        SetItem(index, kDirectionToFirefly[new_dir], -1);
        MoveItem(index, new_dir);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(IndexFromAction(index, new_dir));
    } else if (IsType(index, kElEmpty, action)) {
        SetItem(index, kDirectionToFirefly[action], -1);
        MoveItem(index, action);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(IndexFromAction(index, action));
    } else {
        // No other options, rotate right
        SetItem(index, kDirectionToFirefly[kRotateRight[action]], -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    }
}

void RNDGameState::UpdateButterfly(int index, int action) {
    int new_dir = kRotateRight[action];
    if (IsTypeAdjacent(index, kElAgent) || IsTypeAdjacent(index, kElBlob)) {
        // Explode if touching the agent/blob
        auto it = kElementToExplosion.find(GetItem(index));
        Explode(index, (it == kElementToExplosion.end()) ? kElExplosionEmpty : it->second);
    } else if (IsType(index, kElEmpty, new_dir)) {
        // Butterflies always try to rotate right, otherwise continue forward
        SetItem(index, kDirectionToButterfly[new_dir], -1);
        MoveItem(index, new_dir);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(IndexFromAction(index, new_dir));
    } else if (IsType(index, kElEmpty, action)) {
        SetItem(index, kDirectionToButterfly[action], -1);
        MoveItem(index, action);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(IndexFromAction(index, action));
    } else {
        // No other options, rotate right
        SetItem(index, kDirectionToButterfly[kRotateLeft[action]], -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    }
}

void RNDGameState::UpdateOrange(int index, int action) {
    if (IsType(index, kElEmpty, action)) {
        // Continue moving in direction
        MoveItem(index, action);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(IndexFromAction(index, action));
    } else if (IsTypeAdjacent(index, kElAgent)) {
        // Run into the agent, explode!
        auto it = kElementToExplosion.find(GetItem(index));
        Explode(index, (it == kElementToExplosion.end()) ? kElExplosionEmpty : it->second);
    } else {
        // Blocked, roll for new direction
        std::vector<int> open_dirs;
        for (int dir = 0; dir < kNumActions; ++dir) {
            if (dir == Directions::kNoop || !InBounds(index, dir)) {
                continue;
            }
            if (IsType(index, kElEmpty, dir)) {
                open_dirs.push_back(dir);
            }
        }
        // Roll available directions
        if (!open_dirs.empty()) {
            int new_dir = open_dirs[shared_state_ptr->dist(shared_state_ptr->gen) % open_dirs.size()];
            SetItem(index, kDirectionToOrange[new_dir], -1);
            if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
        }
    }
}

void RNDGameState::UpdateMagicWall(int index) {
    // Dorminant, active, then expired once time runs out
    if (local_state.magic_active) {
        SetItem(index, kElWallMagicOn, -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    } else if (local_state.magic_wall_steps > 0) {
        SetItem(index, kElWallMagicDormant, -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
    } else {
        SetItem(index, kElWallMagicExpired, -1);
    }
}

void RNDGameState::UpdateBlob(int index) {
    // Replace blobs if swap element set
    if (local_state.blob_swap != ElementToItem(kNullElement)) {
        SetItem(index, kCellTypeToElement[local_state.blob_swap + 1], -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(index);
        return;
    }
    ++local_state.blob_size;
    // Check if at least one tile blob can grow to
    if (IsTypeAdjacent(index, kElEmpty) || IsTypeAdjacent(index, kElDirt)) {
        local_state.blob_enclosed = false;
    }
    // Roll if to grow and direction
    bool will_grow = (shared_state_ptr->dist(shared_state_ptr->gen) % 256) < shared_state_ptr->blob_chance;
    int grow_dir = shared_state_ptr->dist(shared_state_ptr->gen) % kNumActions;
    if (will_grow && (IsType(index, kElEmpty, grow_dir) || IsType(index, kElDirt, grow_dir))) {
        SetItem(index, kElBlob, grow_dir, -1);
        if(board.is_opt_queue_event) board.need_update_index_temp.push_back(IndexFromAction(index, grow_dir));        
    }
}

void RNDGameState::UpdateExplosions(int index) {
    SetItem(index, kExplosionToElement.at(GetItem(index)), -1);
}

void RNDGameState::OpenGate(const Element &element) {
    std::vector<int> closed_gate_indices = board.find_all(ElementToItem(element));
    for (const auto &index : closed_gate_indices) {
        SetItem(index, kGateOpenMap.at(GetItem(index)), -1);
    }
}

// ---------------------------------------------------------------------------

void RNDGameState::StartScan() {
    if (local_state.steps_remaining > 0) {
        local_state.steps_remaining += -1;
    }
    local_state.current_reward = 0;
    local_state.blob_size = 0;
    local_state.blob_enclosed = true;
    local_state.reward_signal = 0;
    board.reset_updated();
}

void RNDGameState::EndScan() {
    if (local_state.blob_swap == ElementToItem(kNullElement)) {
        if (local_state.blob_enclosed) {
            local_state.blob_swap = ElementToItem(kElDiamond);
        }
        if (local_state.blob_size > shared_state_ptr->blob_max_size) {
            local_state.blob_swap = ElementToItem(kElStone);
        }
    }
    if (local_state.magic_active) {
        local_state.magic_wall_steps = std::max(local_state.magic_wall_steps - 1, 0);
    }
    local_state.magic_active = local_state.magic_active && (local_state.magic_wall_steps > 0);
}

}    // namespace stonesngems