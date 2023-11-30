#!/usr/bin/python3

from place_token import robot_play_token_in_column
from object_control import TokenColor
from demo_ee import MovableRobot

def main():
    robot = MovableRobot()

    def drop_token_callback():
        pass
    def lodge_token_callback():
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
