GRID_SIZE = (25, 25, 25)
DEPOT_POS = (0,0,0)#(GRID_SIZE[0] - 1, GRID_SIZE[1] - 1, 0)
NUM_ROBOTS = 50
STEP_DELAY = 0.01
STRUCTURES = ["preset_test"] # Valid Structures: preset_test, cube, wall, pyramid, random, overhang, overhangs
TEST_PRESET_NUM = 4 # Only used with STRUCTURES=preset_test, Valid Preset Nums: 1, 2, 3, 4, 5
RANDOM_VOXELS = 100 # Only used with STRUCTURES=random
RANDOM_STRUCTURE_ORIGINS = 1 # Only used with STRUCTURES=random
SCAFFOLDING = True