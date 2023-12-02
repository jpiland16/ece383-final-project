#!/usr/bin/python3

from place_token import robot_play_token_in_column
from object_control import TokenColor
from demo_ee import MovableRobot

def main():
    robot = MovableRobot()

    def drop_token_callback():
        # Return True if token should be played again after being dropped on the ground by robot
        print("Dropped token! Retrying...")
        return True
    def lodge_token_callback():
        # Return True if token should be played again if it gets stuck in the board by robot
        print("A token got stuck or is in the wrong place... ignoring...")
        return False

    for row in range(6):
        for column in range(7):
            robot_play_token_in_column(
                robot,
                column + 1,
                row + 1,
                list(TokenColor)[column % 2], # alternate colors,
                drop_token_callback,
                lodge_token_callback
            )

if __name__ == "__main__":
    main()
