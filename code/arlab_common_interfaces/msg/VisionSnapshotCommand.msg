# Vision snapshot command message

# Available model type constants
uint8 MODEL_GENERAL_OBJECTS_AND_PEOPLE=0
uint8 MODEL_DISHWASHER=2
uint8 MODEL_LAUNDRY=3


# Clear the database before taking snapshot
bool clear_database

# Mask the hand in the snapshot
bool mask_hand

# Extra specialized models to run alongside always-on models (general + people).
# Use the MODEL_* constants above to populate this list.
uint8[] extra_models
