#!/usr/bin/python3

from place_token import robot_play_token_in_column
from object_control import TokenColor
from demo_ee import MovableRobot

def main():
    robot = MovableRobot()

    def drop_token_callback():
        # Return True if token should be played again after being dropped on the ground by robot
        pass
    def lodge_token_callback():
        # Return True if token should be played again if it gets stuck in the board by robot
        pass

    for column in range(7):
        robot_play_token_in_column(
            robot,
            column + 1,
            2,
            list(TokenColor)[column % 2], # alternate colors,
            drop_token_callback,
            lodge_token_callback
        )

if __name__ == "__main__":
    main()
