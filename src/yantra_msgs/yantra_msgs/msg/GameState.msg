# This message provides all information from the game controller
# for additional information see documentation of the game controller
# https://github.com/bhuman/GameController


std_msgs/Header header

#uint8 GAMESTATE_INITAL=0
#uint8 GAMESTATE_READY=1
#uint8 GAMESTATE_SET=2
#uint8 GAMESTATE_PLAYING=3
#uint8 GAMESTATE_FINISHED=4
uint8 game_state

#uint8 STATE_NORMAL = 0
#uint8 STATE_PENALTYSHOOT = 1
#uint8 STATE_OVERTIME = 2
#uint8 STATE_TIMEOUT = 3
#uint8 STATE_FREEKICK = 4
#uint8 STATE_PENALTYKICK = 5
uint8 secondary_state

# For newest version of game controller
# Tells which team has the free kick or penalty kick
uint8 secondary_state_team

bool first_half
uint8 own_score
uint8 rival_score

# Seconds remaining for the game half
int16 seconds_remaining
# Seconds remaining for things like kickoff
uint16 secondary_seconds_remaining

#uint8 NONE=0
#uint8 PENALTY_HL_KID_BALL_MANIPULATION=30
#uint8 PENALTY_HL_KID_PHYSICAL_CONTACT=31
#uint8 PENALTY_HL_KID_ILLEGAL_ATTACK=22
#uint8 PENALTY_HL_KID_ILLEGAL_DEFENSE=4
#uint8 PENALTY_HL_KID_REQUEST_FOR_PICKUP=34
#uint8 PENALTY_HL_KID_REQUEST_FOR_SERVICE=35
uint8 MANUAL=15

uint8 penalty
bool has_kick_off
bool penalized
uint16 seconds_till_unpenalized

# Allowed to move is different from penalized.
# You can for example be not allowed to move due to the current state of the game
bool allowed_to_move

bool drop_in_team
uint16 drop_in_time

uint8 penalty_shot
uint16 single_shots

string coach_message


