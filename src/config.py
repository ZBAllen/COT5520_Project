GRID_SIZE = (30, 30, 10)
DEPOT_POS = (0,0,0)#(GRID_SIZE[0] - 1, GRID_SIZE[1] - 1, 0)
NUM_ROBOTS = 10
STEP_DELAY = 0.01
STRUCTURES = ["preset_test"] # Valid Structures: preset_test, cube, wall, pyramid, random, overhang, overhangs
TEST_PRESET_NUM = 2 # Only used with STRUCTURES=preset_test, Valid Preset Nums: 1, 2, 3, 4, 5
RANDOM_VOXELS = 50 # Only used with STRUCTURES=random
RANDOM_STRUCTURE_ORIGINS = 3 # Only used with STRUCTURES=random
SCAFFOLDING = True