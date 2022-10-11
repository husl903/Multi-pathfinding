#ifndef STONESNGEMS_DEFS_H
#define STONESNGEMS_DEFS_H

#include <array>
#include <cassert>
#include <cstdint>
#include <string>
#include <tuple>
#include <unordered_map>
#include <variant>
#include <vector>
#include <queue>

namespace stonesngems {

// typedefs
using Offset = std::array<int, 2>;

enum class HiddenCellType : int8_t {
    kNull = -1,
    kAgent = 0,
    kEmpty = 1,
    kDirt = 2,
    kStone = 3,
    kStoneFalling = 4,
    kDiamond = 5,
    kDiamondFalling = 6,
    kExitClosed = 7,
    kExitOpen = 8,
    kAgentInExit = 9,
    kFireflyUp = 10,
    kFireflyLeft = 11,
    kFireflyDown = 12,
    kFireflyRight = 13,
    kButterflyUp = 14,
    kButterflyLeft = 15,
    kButterflyDown = 16,
    kButterflyRight = 17,
    kWallBrick = 18,
    kWallSteel = 19,
    kWallMagicDormant = 20,
    kWallMagicOn = 21,
    kWallMagicExpired = 22,
    kBlob = 23,
    kExplosionDiamond = 24,
    kExplosionBoulder = 25,
    kExplosionEmpty = 26,
    kGateRedClosed = 27,
    kGateRedOpen = 28,
    kKeyRed = 29,
    kGateBlueClosed = 30,
    kGateBlueOpen = 31,
    kKeyBlue = 32,
    kGateGreenClosed = 33,
    kGateGreenOpen = 34,
    kKeyGreen = 35,
    kGateYellowClosed = 36,
    kGateYellowOpen = 37,
    kKeyYellow = 38,
    kNut = 39,
    kNutFalling = 40,
    kBomb = 41,
    kBombFalling = 42,
    kOrangeUp = 43,
    kOrangeLeft = 44,
    kOrangeDown = 45,
    kOrangeRight = 46,
    kPebbleInDirt = 47,
    kStoneInDirt = 48,
    kVoidInDirt = 49,
};

// Cell types which are observable
enum class VisibleCellType : int8_t {
    kNull = -1,
    kAgent = 0,
    kEmpty = 1,
    kDirt = 2,
    kStone = 3,
    kDiamond = 4,
    kExitClosed = 5,
    kExitOpen = 6,
    kAgentInExit = 7,
    kFirefly = 8,
    kButterfly = 9,
    kWallBrick = 10,
    kWallSteel = 11,
    kWallMagicOff = 12,
    kWallMagicOn = 13,
    kBlob = 14,
    kExplosion = 15,
    kGateRedClosed = 16,
    kGateRedOpen = 17,
    kKeyRed = 18,
    kGateBlueClosed = 19,
    kGateBlueOpen = 20,
    kKeyBlue = 21,
    kGateGreenClosed = 22,
    kGateGreenOpen = 23,
    kKeyGreen = 24,
    kGateYellowClosed = 25,
    kGateYellowOpen = 26,
    kKeyYellow = 27,
    kNut = 28,
    kBomb = 29,
    kOrange = 30,
    kPebbleInDirt = 31,
    kStoneInDirt = 32,
    kVoidInDirt = 33,
};

constexpr int kNumHiddenCellType = 50;
constexpr int kNumVisibleCellType = 34;

// Directions the interactions take place
enum Directions {
    kNoop = 0,
    kUp = 1,
    kRight = 2,
    kDown = 3,
    kLeft = 4,
    kUpRight = 5,
    kDownRight = 6,
    kDownLeft = 7,
    kUpLeft = 8
};

// Agent can only take a subset of all directions
constexpr int kNumDirections = 9;
constexpr int kNumActions = 5;

enum RewardCodes {
    kRewardAgentDies = 1 << 0,
    kRewardCollectDiamond = 1 << 1,
    kRewardWalkThroughExit = 1 << 2,
    kRewardNutToDiamond = 1 << 3,
    kRewardCollectKey = 1 << 4,
    kRewardCollectKeyRed = 1 << 5,
    kRewardCollectKeyBlue = 1 << 6,
    kRewardCollectKeyGreen = 1 << 7,
    kRewardCollectKeyYellow = 1 << 8,
    kRewardWalkThroughGate = 1 << 9,
    kRewardWalkThroughGateRed = 1 << 10,
    kRewardWalkThroughGateBlue = 1 << 11,
    kRewardWalkThroughGateGreen = 1 << 12,
    kRewardWalkThroughGateYellow = 1 << 13,
};

const std::unordered_map<HiddenCellType, uint8_t> kPointMap = {
    {HiddenCellType::kDiamond, 1},
    {HiddenCellType::kDiamondFalling, 1},
    {HiddenCellType::kAgentInExit, 10},
};

const std::unordered_map<HiddenCellType, RewardCodes> kElementToRewardMap = {
    {HiddenCellType::kDiamond, kRewardCollectDiamond},
    {HiddenCellType::kDiamondFalling, kRewardCollectDiamond},
    {HiddenCellType::kNut, kRewardNutToDiamond},
    {HiddenCellType::kNutFalling, kRewardNutToDiamond},
    {HiddenCellType::kExitOpen, kRewardWalkThroughExit},
    {HiddenCellType::kKeyRed, kRewardCollectKeyRed},
    {HiddenCellType::kKeyBlue, kRewardCollectKeyBlue},
    {HiddenCellType::kKeyGreen, kRewardCollectKeyGreen},
    {HiddenCellType::kKeyYellow, kRewardCollectKeyYellow},
    {HiddenCellType::kGateRedOpen, kRewardWalkThroughGateRed},
    {HiddenCellType::kGateBlueOpen, kRewardWalkThroughGateBlue},
    {HiddenCellType::kGateGreenOpen, kRewardWalkThroughGateGreen},
    {HiddenCellType::kGateYellowOpen, kRewardWalkThroughGateYellow},
};

constexpr int kAgentPosExit = -1;
constexpr int kAgentPosDie = -2;

// Element entities, along with properties
struct Element {
    HiddenCellType cell_type;
    VisibleCellType visible_type;
    int properties;
    char id;
    bool has_updated;

    Element()
        : cell_type(HiddenCellType::kNull),
          visible_type(VisibleCellType::kNull),
          properties(0),
          id(0),
          has_updated(false) {}

    Element(HiddenCellType cell_type, VisibleCellType visible_type, int properties, char id)
        : cell_type(cell_type), visible_type(visible_type), properties(properties), id(id), has_updated(false) {}

    bool operator==(const Element &rhs) const {
        return this->cell_type == rhs.cell_type;
    }

    bool operator!=(const Element &rhs) const {
        return this->cell_type != rhs.cell_type;
    }
};

// Property bit flags
enum ElementProperties {
    kNone = 0,
    kConsumable = 1 << 0,
    kCanExplode = 1 << 1,
    kRounded = 1 << 2,
    kTraversable = 1 << 3,
    kPushable = 1 << 4,
};

// Default base element
const Element kNullElement = {
    HiddenCellType::kNull,
    VisibleCellType::kNull,
    -1,
    0,
};

// All possible elements
const Element kElAgent{
    HiddenCellType::kAgent,
    VisibleCellType::kAgent,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    '@',
};
const Element kElAgentInExit = {
    HiddenCellType::kAgentInExit,
    VisibleCellType::kAgentInExit,
    ElementProperties::kNone,
    '!',
};
const Element kElExitOpen = {
    HiddenCellType::kExitOpen,
    VisibleCellType::kExitOpen,
    ElementProperties::kTraversable,
    '#',
};
const Element kElExitClosed = {
    HiddenCellType::kExitClosed,
    VisibleCellType::kExitClosed,
    ElementProperties::kNone,
    'C',
};
const Element kElEmpty = {
    HiddenCellType::kEmpty,
    VisibleCellType::kEmpty,
    ElementProperties::kConsumable | ElementProperties::kTraversable,
    ' ',
};
const Element kElDirt = {
    HiddenCellType::kDirt,
    VisibleCellType::kDirt,
    ElementProperties::kConsumable | ElementProperties::kTraversable,
    '.',
};
const Element kElStone = {
    HiddenCellType::kStone,
    VisibleCellType::kStone,
    ElementProperties::kConsumable | ElementProperties::kRounded | ElementProperties::kPushable,
    'o',
};
const Element kElStoneFalling = {
    HiddenCellType::kStoneFalling,
    VisibleCellType::kStone,
    ElementProperties::kConsumable,
    'o',
};
const Element kElDiamond = {
    HiddenCellType::kDiamond,
    VisibleCellType::kDiamond,
    ElementProperties::kConsumable | ElementProperties::kRounded | ElementProperties::kTraversable,
    '*',
};
const Element kElDiamondFalling = {
    HiddenCellType::kDiamondFalling,
    VisibleCellType::kDiamond,
    ElementProperties::kConsumable,
    '*',
};
const Element kElFireflyUp = {
    HiddenCellType::kFireflyUp,
    VisibleCellType::kFirefly,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'F',
};
const Element kElFireflyLeft = {
    HiddenCellType::kFireflyLeft,
    VisibleCellType::kFirefly,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'F',
};
const Element kElFireflyDown = {
    HiddenCellType::kFireflyDown,
    VisibleCellType::kFirefly,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'F',
};
const Element kElFireflyRight = {
    HiddenCellType::kFireflyRight,
    VisibleCellType::kFirefly,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'F',
};
const Element kElButterflyUp = {
    HiddenCellType::kButterflyUp,
    VisibleCellType::kButterfly,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'U',
};
const Element kElButterflyLeft = {
    HiddenCellType::kButterflyLeft,
    VisibleCellType::kButterfly,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'U',
};
const Element kElButterflyDown = {
    HiddenCellType::kButterflyDown,
    VisibleCellType::kButterfly,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'U',
};
const Element kElButterflyRight = {
    HiddenCellType::kButterflyRight,
    VisibleCellType::kButterfly,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'U',
};
const Element kElBlob = {
    HiddenCellType::kBlob,
    VisibleCellType::kBlob,
    ElementProperties::kConsumable,
    'A',
};
const Element kElWallBrick = {
    HiddenCellType::kWallBrick,
    VisibleCellType::kWallBrick,
    ElementProperties::kConsumable | ElementProperties::kRounded,
    'H',
};
const Element kElWallSteel = {
    HiddenCellType::kWallSteel,
    VisibleCellType::kWallSteel,
    ElementProperties::kNone,
    'S',
};
const Element kElWallMagicOn = {
    HiddenCellType::kWallMagicOn,
    VisibleCellType::kWallMagicOn,
    ElementProperties::kConsumable,
    'M',
};
const Element kElWallMagicDormant = {
    HiddenCellType::kWallMagicDormant,
    VisibleCellType::kWallMagicOff,
    ElementProperties::kConsumable,
    'Q',
};
const Element kElWallMagicExpired = {
    HiddenCellType::kWallMagicExpired,
    VisibleCellType::kWallMagicOff,
    ElementProperties::kConsumable,
    'Q',
};
const Element kElExplosionDiamond = {
    HiddenCellType::kExplosionDiamond,
    VisibleCellType::kExplosion,
    ElementProperties::kNone,
    'E',
};
const Element kElExplosionBoulder = {
    HiddenCellType::kExplosionBoulder,
    VisibleCellType::kExplosion,
    ElementProperties::kNone,
    'E',
};
const Element kElExplosionEmpty = {
    HiddenCellType::kExplosionEmpty,
    VisibleCellType::kExplosion,
    ElementProperties::kNone,
    'E',
};
const Element kElGateRedClosed = {
    HiddenCellType::kGateRedClosed,
    VisibleCellType::kGateRedClosed,
    ElementProperties::kNone,
    'r',
};
const Element kElGateRedOpen = {
    HiddenCellType::kGateRedOpen,
    VisibleCellType::kGateRedOpen,
    ElementProperties::kNone,
    'R',
};
const Element kElKeyRed = {
    HiddenCellType::kKeyRed,
    VisibleCellType::kKeyRed,
    ElementProperties::kTraversable,
    '1',
};
const Element kElGateBlueClosed = {
    HiddenCellType::kGateBlueClosed,
    VisibleCellType::kGateBlueClosed,
    ElementProperties::kNone,
    'b',
};
const Element kElGateBlueOpen = {
    HiddenCellType::kGateBlueOpen,
    VisibleCellType::kGateBlueOpen,
    ElementProperties::kNone,
    'B',
};
const Element kElKeyBlue = {
    HiddenCellType::kKeyBlue,
    VisibleCellType::kKeyBlue,
    ElementProperties::kTraversable,
    '2',
};
const Element kElGateGreenClosed = {
    HiddenCellType::kGateGreenClosed,
    VisibleCellType::kGateGreenClosed,
    ElementProperties::kNone,
    'g',
};
const Element kElGateGreenOpen = {
    HiddenCellType::kGateGreenOpen,
    VisibleCellType::kGateGreenOpen,
    ElementProperties::kNone,
    'G',
};
const Element kElKeyGreen = {
    HiddenCellType::kKeyGreen,
    VisibleCellType::kKeyGreen,
    ElementProperties::kTraversable,
    '3',
};
const Element kElGateYellowClosed = {
    HiddenCellType::kGateYellowClosed,
    VisibleCellType::kGateYellowClosed,
    ElementProperties::kNone,
    'y',
};
const Element kElGateYellowOpen = {
    HiddenCellType::kGateYellowOpen,
    VisibleCellType::kGateYellowOpen,
    ElementProperties::kNone,
    'Y',
};
const Element kElKeyYellow = {
    HiddenCellType::kKeyYellow,
    VisibleCellType::kKeyYellow,
    ElementProperties::kTraversable,
    '4',
};
const Element kElNut = {
    HiddenCellType::kNut,
    VisibleCellType::kNut,
    ElementProperties::kRounded | ElementProperties::kConsumable | ElementProperties::kPushable,
    '+',
};
const Element kElNutFalling = {
    HiddenCellType::kNutFalling,
    VisibleCellType::kNut,
    ElementProperties::kRounded | ElementProperties::kConsumable,
    '+',
};
const Element kElBomb = {
    HiddenCellType::kBomb,
    VisibleCellType::kBomb,
    ElementProperties::kRounded | ElementProperties::kConsumable | ElementProperties::kCanExplode |
        ElementProperties::kPushable,
    '^',
};
const Element kElBombFalling = {
    HiddenCellType::kBombFalling,
    VisibleCellType::kBomb,
    ElementProperties::kRounded | ElementProperties::kConsumable | ElementProperties::kCanExplode,
    '^',
};
const Element kElOrangeUp = {
    HiddenCellType::kOrangeUp,
    VisibleCellType::kOrange,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'X',
};
const Element kElOrangeLeft = {
    HiddenCellType::kOrangeLeft,
    VisibleCellType::kOrange,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'X',
};
const Element kElOrangeDown = {
    HiddenCellType::kOrangeDown,
    VisibleCellType::kOrange,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'X',
};
const Element kElOrangeRight = {
    HiddenCellType::kOrangeRight,
    VisibleCellType::kOrange,
    ElementProperties::kConsumable | ElementProperties::kCanExplode,
    'X',
};

// Hash for Element, so we can use as a map key
struct ElementHash {
    std::size_t operator()(const Element &e) const {
        return static_cast<int>(e.cell_type) - static_cast<int>(HiddenCellType::kNull);
    }
};

// ----- Conversion maps -----
// Swap map for from cell type id to element
const std::array<Element, kNumHiddenCellType + 1> kCellTypeToElement{
    kNullElement,           // HiddenCellType::kNull
    kElAgent,               // HiddenCellType::kAgent
    kElEmpty,               // HiddenCellType::kEmpty
    kElDirt,                // HiddenCellType::kDirt
    kElStone,               // HiddenCellType::kStone
    kElStoneFalling,        // HiddenCellType::kStoneFalling
    kElDiamond,             // HiddenCellType::kDiamond
    kElDiamondFalling,      // HiddenCellType::kDiamondFalling
    kElExitClosed,          // HiddenCellType::kExitClosed
    kElExitOpen,            // HiddenCellType::kExitOpen
    kElAgentInExit,         // HiddenCellType::kAgentInExit
    kElFireflyUp,           // HiddenCellType::kAgentInExit
    kElFireflyLeft,         // HiddenCellType::kFireflyLeft
    kElFireflyDown,         // HiddenCellType::kFireflyDown
    kElFireflyRight,        // HiddenCellType::kFireflyRight
    kElButterflyUp,         // HiddenCellType::kButterflyUp
    kElButterflyLeft,       // HiddenCellType::kButterflyLeft
    kElButterflyDown,       // HiddenCellType::kButterflyDown
    kElButterflyRight,      // HiddenCellType::kButterflyRight
    kElWallBrick,           // HiddenCellType::kWallBrick
    kElWallSteel,           // HiddenCellType::kWallSteel
    kElWallMagicOn,         // HiddenCellType::kWallMagicOn
    kElWallMagicDormant,    // HiddenCellType::kWallMagicDormant
    kElWallMagicExpired,    // HiddenCellType::kWallMagicExpired
    kElBlob,                // HiddenCellType::kBlob
    kElExplosionDiamond,    // HiddenCellType::kExplosionDiamond
    kElExplosionBoulder,    // HiddenCellType::kExplosionBoulder
    kElExplosionEmpty,      // HiddenCellType::kExplosionEmpty
    kElGateRedClosed,       // HiddenCellType::kGateRedClosed
    kElGateRedOpen,         // HiddenCellType::kGateRedOpen
    kElKeyRed,              // HiddenCellType::kKeyRed
    kElGateBlueClosed,      // HiddenCellType::kGateBlueClosed
    kElGateBlueOpen,        // HiddenCellType::kGateBlueOpen
    kElKeyBlue,             // HiddenCellType::kKeyBlue
    kElGateGreenClosed,     // HiddenCellType::kGateGreenClosed
    kElGateGreenOpen,       // HiddenCellType::kGateGreenOpen
    kElKeyGreen,            // HiddenCellType::kKeyGreen
    kElGateYellowClosed,    // HiddenCellType::kGateYellowClosed
    kElGateYellowOpen,      // HiddenCellType::kGateYellowOpen
    kElKeyYellow,           // HiddenCellType::kKeyYellow
    kElNut,                 // HiddenCellType::kNut
    kElNutFalling,          // HiddenCellType::kNutFalling
    kElBomb,                // HiddenCellType::kBomb
    kElBombFalling,         // HiddenCellType::kBombFalling
    kElOrangeUp,            // HiddenCellType::kOrangeUp
    kElOrangeLeft,          // HiddenCellType::kOrangeLeft
    kElOrangeDown,          // HiddenCellType::kOrangeDown
    kElOrangeRight,         // HiddenCellType::kOrangeRight
    kNullElement,           // HiddenCellType::kPebbleInDirt
    kNullElement,           // HiddenCellType::kStoneInDirt
    kNullElement,           // HiddenCellType::kVoidInDirt
};

// Swap map for from cell type id to string for debugging
const std::unordered_map<int8_t, std::string> kCellTypeToString{
    {static_cast<int8_t>(HiddenCellType::kNull), "NullElement"},
    {static_cast<int8_t>(HiddenCellType::kAgent), "Agent"},
    {static_cast<int8_t>(HiddenCellType::kEmpty), "Empty"},
    {static_cast<int8_t>(HiddenCellType::kDirt), "Dirt"},
    {static_cast<int8_t>(HiddenCellType::kStone), "Stone"},
    {static_cast<int8_t>(HiddenCellType::kStoneFalling), "StoneFalling"},
    {static_cast<int8_t>(HiddenCellType::kDiamond), "Diamond"},
    {static_cast<int8_t>(HiddenCellType::kDiamondFalling), "DiamondFalling"},
    {static_cast<int8_t>(HiddenCellType::kExitClosed), "ExitClosed"},
    {static_cast<int8_t>(HiddenCellType::kExitOpen), "ExitOpen"},
    {static_cast<int8_t>(HiddenCellType::kAgentInExit), "AgentInExit"},
    {static_cast<int8_t>(HiddenCellType::kFireflyUp), "FireflyUp"},
    {static_cast<int8_t>(HiddenCellType::kFireflyLeft), "FireflyLeft"},
    {static_cast<int8_t>(HiddenCellType::kFireflyDown), "FireflyDown"},
    {static_cast<int8_t>(HiddenCellType::kFireflyRight), "FireflyRight"},
    {static_cast<int8_t>(HiddenCellType::kButterflyUp), "ButterflyUp"},
    {static_cast<int8_t>(HiddenCellType::kButterflyLeft), "ButterflyLeft"},
    {static_cast<int8_t>(HiddenCellType::kButterflyDown), "ButterflyDown"},
    {static_cast<int8_t>(HiddenCellType::kButterflyRight), "ButterflyRight"},
    {static_cast<int8_t>(HiddenCellType::kWallBrick), "WallBrick"},
    {static_cast<int8_t>(HiddenCellType::kWallSteel), "WallSteel"},
    {static_cast<int8_t>(HiddenCellType::kWallMagicOn), "WallMagicOn"},
    {static_cast<int8_t>(HiddenCellType::kWallMagicDormant), "WallMagicDormant"},
    {static_cast<int8_t>(HiddenCellType::kWallMagicExpired), "WallMagicExpired"},
    {static_cast<int8_t>(HiddenCellType::kBlob), "Blob"},
    {static_cast<int8_t>(HiddenCellType::kExplosionBoulder), "ExplosionBoulder"},
    {static_cast<int8_t>(HiddenCellType::kExplosionDiamond), "ExplosionDiamond"},
    {static_cast<int8_t>(HiddenCellType::kExplosionEmpty), "ExplosionEmpty"},
    {static_cast<int8_t>(HiddenCellType::kGateRedClosed), "GateRedClosed"},
    {static_cast<int8_t>(HiddenCellType::kGateRedOpen), "GateRedOpen"},
    {static_cast<int8_t>(HiddenCellType::kKeyRed), "KeyRed"},
    {static_cast<int8_t>(HiddenCellType::kGateBlueClosed), "GateBlueClosed"},
    {static_cast<int8_t>(HiddenCellType::kGateBlueOpen), "GateBlueOpen"},
    {static_cast<int8_t>(HiddenCellType::kKeyBlue), "KeyBlue"},
    {static_cast<int8_t>(HiddenCellType::kGateGreenClosed), "GateGreenClosed"},
    {static_cast<int8_t>(HiddenCellType::kGateGreenOpen), "GateGreenOpen"},
    {static_cast<int8_t>(HiddenCellType::kKeyGreen), "KeyGreen"},
    {static_cast<int8_t>(HiddenCellType::kGateYellowClosed), "GateYellowClosed"},
    {static_cast<int8_t>(HiddenCellType::kGateYellowOpen), "GateYellowOpen"},
    {static_cast<int8_t>(HiddenCellType::kKeyYellow), "KeyYellow"},
    {static_cast<int8_t>(HiddenCellType::kNut), "Nut"},
    {static_cast<int8_t>(HiddenCellType::kNutFalling), "NutFalling"},
    {static_cast<int8_t>(HiddenCellType::kBomb), "Bomb"},
    {static_cast<int8_t>(HiddenCellType::kBombFalling), "BombFalling"},
    {static_cast<int8_t>(HiddenCellType::kOrangeUp), "OrangeUp"},
    {static_cast<int8_t>(HiddenCellType::kOrangeLeft), "OrangeLeft"},
    {static_cast<int8_t>(HiddenCellType::kOrangeDown), "OrangeDown"},
    {static_cast<int8_t>(HiddenCellType::kOrangeRight), "OrangeRight"},
};

// Rotate actions right
const std::array<Directions, kNumActions> kRotateRight{Directions::kNoop, Directions::kRight, Directions::kDown,
                                                       Directions::kLeft, Directions::kUp};

// Rotate actions left
const std::array<Directions, kNumActions> kRotateLeft{Directions::kNoop, Directions::kLeft, Directions::kUp,
                                                      Directions::kRight, Directions::kDown};

// actions to strings
const std::unordered_map<int, std::string> kActionsToString{
    {Directions::kUp, "up"},       {Directions::kLeft, "left"}, {Directions::kDown, "down"},
    {Directions::kRight, "right"}, {Directions::kNoop, "none"},
};

// directions to offsets (col, row)
const std::array<Offset, kNumDirections> kDirectionOffsets{{
    {0, 0},     // Directions::kNoop
    {0, -1},    // Directions::kUp
    {1, 0},     // Directions::kRight
    {0, 1},     // Directions::kDown
    {-1, 0},    // Directions::kLeft
    {1, -1},    // Directions::kUpRight
    {1, 1},     // Directions::kDownRight
    {-1, 1},    // Directions::kDownLeft
    {-1, -1}    // Directions::kUpLeft
}};

// Directions to fireflys
const std::array<Element, kNumActions> kDirectionToFirefly{
    kNullElement,       // Directions::kNoop  (shouldn't happen)
    kElFireflyUp,       // Directions::kUp
    kElFireflyRight,    // Directions::kRight
    kElFireflyDown,     // Directions::kDown
    kElFireflyLeft,     // Directions::kLeft
};

// Firefly to directions
const std::unordered_map<Element, int, ElementHash> kFireflyToDirection{
    {kElFireflyUp, Directions::kUp},
    {kElFireflyLeft, Directions::kLeft},
    {kElFireflyDown, Directions::kDown},
    {kElFireflyRight, Directions::kRight},
};

// Directions to butterflys
const std::array<Element, kNumActions> kDirectionToButterfly{
    kNullElement,         // Directions::kNoop  (shouldn't happen)
    kElButterflyUp,       // Directions::kUp
    kElButterflyRight,    // Directions::kRight
    kElButterflyDown,     // Directions::kDown
    kElButterflyLeft,     // Directions::kLeft
};

// Butterfly to directions
const std::unordered_map<Element, int, ElementHash> kButterflyToDirection{
    {kElButterflyUp, Directions::kUp},
    {kElButterflyLeft, Directions::kLeft},
    {kElButterflyDown, Directions::kDown},
    {kElButterflyRight, Directions::kRight},
};

// Orange to directions
const std::unordered_map<Element, int, ElementHash> kOrangeToDirection{
    {kElOrangeUp, Directions::kUp},
    {kElOrangeLeft, Directions::kLeft},
    {kElOrangeDown, Directions::kDown},
    {kElOrangeRight, Directions::kRight},
};

// Direction to Orange
const std::array<Element, kNumActions> kDirectionToOrange{
    kNullElement,      // Directions::kNoop  (shouldn't happen)
    kElOrangeUp,       // Directions::kUp
    kElOrangeRight,    // Directions::kRight
    kElOrangeDown,     // Directions::kDown
    kElOrangeLeft,     // Directions::kLeft
};

// Element explosion maps
const std::unordered_map<Element, Element, ElementHash> kElementToExplosion{
    {kElFireflyUp, kElExplosionEmpty},       {kElFireflyLeft, kElExplosionEmpty},
    {kElFireflyDown, kElExplosionEmpty},     {kElFireflyRight, kElExplosionEmpty},
    {kElButterflyUp, kElExplosionDiamond},   {kElButterflyLeft, kElExplosionDiamond},
    {kElButterflyDown, kElExplosionDiamond}, {kElButterflyRight, kElExplosionDiamond},
    {kElAgent, kElExplosionEmpty},           {kElBomb, kElExplosionEmpty},
    {kElBombFalling, kElExplosionEmpty},     {kElOrangeUp, kElExplosionEmpty},
    {kElOrangeLeft, kElExplosionEmpty},      {kElOrangeDown, kElExplosionEmpty},
    {kElOrangeRight, kElExplosionEmpty},
};

// Explosions back to elements
const std::unordered_map<Element, Element, ElementHash> kExplosionToElement{
    {kElExplosionDiamond, kElDiamond},
    {kElExplosionBoulder, kElStone},
    {kElExplosionEmpty, kElEmpty},
};

// Magic wall conversion map
const std::unordered_map<Element, Element, ElementHash> kMagicWallConversion{
    {kElStoneFalling, kElDiamondFalling},
    {kElDiamondFalling, kElStoneFalling},
};

// Gem point maps
const std::unordered_map<Element, int, ElementHash> kGemPoints{
    {kElDiamond, 10},
    {kElDiamondFalling, 10},
    {kElAgentInExit, 100},
};

// Gate open conversion map
const std::unordered_map<Element, Element, ElementHash> kGateOpenMap{
    {kElGateRedClosed, kElGateRedOpen},
    {kElGateBlueClosed, kElGateBlueOpen},
    {kElGateGreenClosed, kElGateGreenOpen},
    {kElGateYellowClosed, kElGateYellowOpen},
};
// Gate key map
const std::unordered_map<Element, Element, ElementHash> kKeyToGate{
    {kElKeyRed, kElGateRedClosed},
    {kElKeyBlue, kElGateBlueClosed},
    {kElKeyGreen, kElGateGreenClosed},
    {kElKeyYellow, kElGateYellowClosed},
};
// Key signal map
const std::unordered_map<Element, RewardCodes, ElementHash> kKeyToSignal{
    {kElKeyRed, RewardCodes::kRewardCollectKeyRed},
    {kElKeyBlue, RewardCodes::kRewardCollectKeyBlue},
    {kElKeyGreen, RewardCodes::kRewardCollectKeyGreen},
    {kElKeyYellow, RewardCodes::kRewardCollectKeyYellow},
};
// Gate signal map
const std::unordered_map<Element, RewardCodes, ElementHash> kGateToSignal{
    {kElGateRedOpen, RewardCodes::kRewardWalkThroughGateRed},
    {kElGateBlueOpen, RewardCodes::kRewardWalkThroughGateBlue},
    {kElGateGreenOpen, RewardCodes::kRewardWalkThroughGateGreen},
    {kElGateYellowOpen, RewardCodes::kRewardWalkThroughGateYellow},
};

// Stationary to falling
const std::unordered_map<Element, Element, ElementHash> kElToFalling{
    {kElDiamond, kElDiamondFalling},
    {kElStone, kElStoneFalling},
    {kElNut, kElNutFalling},
    {kElBomb, kElBombFalling},
};

// Helper functions
inline bool IsActionHorz(int action) {
    return action == Directions::kLeft || action == Directions::kRight;
}

inline bool IsFirefly(const Element &element) {
    return element == kElFireflyUp || element == kElFireflyLeft || element == kElFireflyDown ||
           element == kElFireflyRight;
}

inline bool IsButterfly(const Element &element) {
    return element == kElButterflyUp || element == kElButterflyLeft || element == kElButterflyDown ||
           element == kElButterflyRight;
}

inline bool IsOrange(const Element &element) {
    return element == kElOrangeUp || element == kElOrangeLeft || element == kElOrangeDown || element == kElOrangeRight;
}

inline bool IsExplosion(const Element &element) {
    return element == kElExplosionBoulder || element == kElExplosionDiamond || element == kElExplosionEmpty;
}

inline bool IsMagicWall(const Element &element) {
    return element == kElWallMagicDormant || element == kElWallMagicExpired || element == kElWallMagicOn;
}

inline bool IsOpenGate(const Element &element) {
    return element == kElGateRedOpen || element == kElGateBlueOpen || element == kElGateGreenOpen ||
           element == kElGateYellowOpen;
}

inline bool IsKey(const Element &element) {
    return element == kElKeyRed || element == kElKeyBlue || element == kElKeyGreen || element == kElKeyYellow;
}

inline std::underlying_type_t<HiddenCellType> ElementToItem(const Element &element) {
    return static_cast<std::underlying_type_t<HiddenCellType>>(element.cell_type);
}

struct Board {
    Board() = delete;
    Board(int rows, int cols, uint8_t gems_required, int max_steps)
        : zorb_hash(0),
          rows(rows),
          cols(cols),
          gems_required(gems_required),
          agent_pos(-1),
          agent_idx(-1),
          max_steps(max_steps),
          grid(rows * cols, 0),
          has_updated(rows * cols, 0) {}

    bool operator==(const Board &other) const {
        return grid == other.grid;
    }

    int8_t &item(int index) {
        assert(index < rows * cols);
        return grid[index];
    }

    int8_t item(int index) const {
        assert(index < rows * cols);
        return grid[index];
    }

    std::vector<int> find_all(int8_t element) const {
        std::vector<int> indices;
        for (int i = 0; i < rows * cols; ++i) {
            if (grid[i] == element) {
                indices.push_back(i);
            }
        }
        return indices;
    }

    int find_single(int8_t element) const {
        for (int i = 0; i < rows * cols; ++i) {
            if (grid[i] == element) {
                return i;
            }
        }
        return -1;
    }

    void reset_updated() {
        for (int i = 0; i < rows * cols; ++i) {
            has_updated[i] = false;
        }
    }

    uint64_t zorb_hash;
    int rows;
    int cols;
    uint8_t gems_required;
    int agent_pos;
    int agent_idx;
    int max_steps;
    std::vector<int8_t> grid;
    std::vector<bool> has_updated;
    std::queue<int> need_update_index;
    bool is_update_event = true;
};

}    // namespace stonesngems

#endif    // STONESNGEMS_DEFS_H