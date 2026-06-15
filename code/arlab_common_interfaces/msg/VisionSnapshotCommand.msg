# Vision snapshot command message

# Available model type constants
uint8 MODEL_GENERAL_OBJECTS=0
uint8 MODEL_PEOPLE=1
uint8 MODEL_WALLS=2
uint8 MODEL_DOOR_OPEN=3
uint8 MODEL_STORAGE_LOCATION=4
uint8 MODEL_KITCHEN_WORKSPACES=5
uint8 MODEL_DISHWASHER_STATUS=6
uint8 MODEL_DISHWASHER_SLOTS=7
uint8 MODEL_WASHING_MACHINE_POINTS=8
uint8 MODEL_TSHIRT_DETECT=9
uint8 MODEL_TSHIRT_SEGMENT=10
uint8 MODEL_WASHING_MACHINE_STATUS=11

# Clear the database before taking snapshot
bool clear_database

# Mask the hand in the snapshot
bool mask_hand

# Extra specialized models to run alongside always-on models (general + people).
# Use the MODEL_* constants above to populate this list.
uint8[] extra_models
