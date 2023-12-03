#!/usr/bin/python3

from place_token import robot_play_token_in_column
from object_control import TokenColor, drop_token_in_column, ObjectController
from attacher import AttachDetachHelper
from demo_ee import MovableRobot
import time, random
from enum import Enum
from typing import List, Optional
import os
from c4solver import Connect4Solver
from image_processing import VideoCamera, get_column_played
from math import floor

def get_integer(msg: str, min: int=None, max: int=None) -> int:
    while True:
        fun_type(msg, newline=False)
        val = input(" > ")
        try:
            val = int(val)
            if ((min == None) or val >= min) and ((max == None) or val <= max):
                return val
            else:
                print(f"Value is out of range! (min = {min}, max = {max})")
        except KeyboardInterrupt:
            break
        except:
            print("Invalid entry!")

def select_option(options: list, delay: float = 0.5):
    _max = len(options) - 1
    for index, option in enumerate(options):
        fun_type(f" {index} - {str(option)}")
    print()
    time.sleep(delay)
    return options[get_integer("Enter an option " + 
                f"(0-{_max})", min = 0, max = _max)]

def fun_type(message: str, delay: float = 0, newline: bool = True):
    for c in message:
        d = random.expovariate(40) # exponential with mean = 40 chars per second
        print(c, end="", flush=True)
        time.sleep(d)
    time.sleep(delay)
    if newline:
        print()

def spin_text(message: str, spin_time: float):
    SPIN_SPEED = 10 # Hz

    spin_delay = 1 / SPIN_SPEED
    count = 0

    spin_chars = ["/", "-", "\\", "|"]
    fun_type(message, newline=False)

    print(" ", end="")
    while count * spin_delay < spin_time:
        print("\b" + spin_chars[count % len(spin_chars)], end="", flush=True)
        time.sleep(spin_delay)
        count += 1
    print("\b \n")

class GameMode(str, Enum):
    USER_FIRST = "Play as red (you will go first)"
    ROBOT_FIRST = "Play as yellow (the robot will go first)"
    RANDOM = "Randomly choose who gets to play first"
    def __str__(self):
        return self.value

class BoardCell():
    def __init__(self, parent: 'Board', token: Optional[TokenColor], column: int, row: int):
        self.parent = parent
        self.token = token
        self.column = column
        self.row = row

    def get_northwest_neighbors_count(self):
        if self.column == 1:
            return 0
        if self.row == 6:
            return 0
        adjacent = self.parent.get_cell(self.column - 1, self.row + 1)
        if adjacent.token == self.token:
            return 1 + adjacent.get_northwest_neighbors_count()
        return 0
        
    def get_north_neighbors_count(self):
        if self.row == 6:
            return 0
        adjacent = self.parent.get_cell(self.column, self.row + 1)
        if adjacent.token == self.token:
            return 1 + adjacent.get_north_neighbors_count()
        return 0
    
    def get_northeast_neighbors_count(self):
        if self.column == 7:
            return 0
        if self.row == 6:
            return 0
        adjacent = self.parent.get_cell(self.column + 1, self.row + 1)
        if adjacent.token == self.token:
            return 1 + adjacent.get_northeast_neighbors_count()
        return 0

    def get_west_neighbors_count(self):
        if self.column == 1:
            return 0
        adjacent = self.parent.get_cell(self.column - 1, self.row)
        if adjacent.token == self.token:
            return 1 + adjacent.get_west_neighbors_count()
        return 0
        
    def get_east_neighbors_count(self):
        if self.column == 7:
            return 0
        adjacent = self.parent.get_cell(self.column + 1, self.row)
        if adjacent.token == self.token:
            return 1 + adjacent.get_east_neighbors_count()
        return 0

    def get_southwest_neighbors_count(self):
        if self.column == 1:
            return 0
        if self.row == 1:
            return 0
        adjacent = self.parent.get_cell(self.column - 1, self.row - 1)
        if adjacent.token == self.token:
            return 1 + adjacent.get_southwest_neighbors_count()
        return 0
        
    def get_south_neighbors_count(self):
        if self.row == 1:
            return 0
        adjacent = self.parent.get_cell(self.column, self.row - 1)
        if adjacent.token == self.token:
            return 1 + adjacent.get_south_neighbors_count()
        return 0
    
    def get_southeast_neighbors_count(self):
        if self.column == 7:
            return 0
        if self.row == 1:
            return 0
        adjacent = self.parent.get_cell(self.column + 1, self.row - 1)
        if adjacent.token == self.token:
            return 1 + adjacent.get_southeast_neighbors_count()
        return 0
        
class InvalidBoardError(ValueError):
    pass

class InvalidMoveError(ValueError):
    pass

class Board():
    def __init__(self, state: str = "") -> None:
        self._cells: List[List[BoardCell]] = [[BoardCell(self, None, col + 1, row + 1) for row in range(6)] for col in range(7)]

        self._column_heights = [0] * 7
        self.winner = None
        self.state = ""

        for char in state:
            self.update_with_char(char)

    def update_with_char(self, char: chr):
            """
            index is 0 for player 1, and 1 for player 2.
            """
            if self.winner is not None:
                raise InvalidMoveError("Game has already been won!")
            

            column = int(char) - 1
            self._cells[column][self._column_heights[column]].token = [TokenColor.RED, TokenColor.YELLOW][len(self.state) % 2]
            self._column_heights[column] += 1
            self.winner = self._check_winner()
            self.state += char

    def _check_winner(self):

        red_win = False
        yellow_win = False

        for i in range(6):
            row = 6 - i
            for j in range(7):
                column = j + 1
                cell = self.get_cell(column, row)

                if cell.token == None:
                    continue

                ns_count = 1 + cell.get_north_neighbors_count() + cell.get_south_neighbors_count()
                ne_sw_count = 1 + cell.get_northeast_neighbors_count() + cell.get_southwest_neighbors_count()
                nw_se_count = 1 + cell.get_northwest_neighbors_count() + cell.get_southeast_neighbors_count()
                ew_count = 1 + cell.get_east_neighbors_count() + cell.get_west_neighbors_count()

                win = ns_count >= 4 or ne_sw_count >= 4 or nw_se_count >= 4 or ew_count >= 4

                if win:
                    if cell.token == TokenColor.RED:
                        red_win = True
                    if cell.token == TokenColor.YELLOW:
                        yellow_win = True

        if not red_win and not yellow_win:
            return None
        
        if red_win and yellow_win:
            self.print()
            raise InvalidBoardError("Board has two winners!")

        if red_win:
            return TokenColor.RED
        
        if yellow_win:
            return TokenColor.YELLOW

    def get_cell(self, column: int, row: int) -> BoardCell:
        if (not (1 <= column <= 7)) or int(column != column):
            raise ValueError(f"Invalid column {column}")
        if (not (1 <= row <= 6)) or int(row != row):
            raise ValueError(f"Invalid row {row}")
        
        return self._cells[column - 1][row - 1]
    
    def print(self):
        for i in range(6):
            row = 6 - i
            for j in range(7):
                column = j + 1
                cell = self.get_cell(column, row)
                if cell.token == TokenColor.RED:
                    print("X", end=" ")
                elif cell.token == TokenColor.YELLOW:
                    print("O", end=" ")
                else:
                    print(" ", end=" ")
            print()

def main():

    robot = MovableRobot()
    object_controller = ObjectController()
    attach_detach_helper = AttachDetachHelper()
    solver = Connect4Solver()
    board = Board()
    video_camera = VideoCamera()

    os.system("clear")
    fun_type("Welcome to the Connect4 AI!", delay=2)
    print()
    fun_type("How would you like to start the game?", delay=0.5)
    print()
    game_mode = select_option(list(GameMode))

    if game_mode == GameMode.RANDOM:
        spin_text("Flipping a coin... ", 1)
        result = int(random.random() < 0.5)
        fun_type(f"The coin came up {['HEADS', 'TAILS'][result]}. That means {['you', 'the robot'][result]} will go first!")
        game_mode = list(GameMode)[result]

    print()
    fun_type("Starting the game...", delay=2)
    print()

    def drop_token_callback():
        # Return True if token should be played again after being dropped on the ground by robot
        return True
    def lodge_token_callback():
        # Return True if token should be played again if it gets stuck in the board by robot
        pass

    if game_mode == GameMode.USER_FIRST:
        user_color = TokenColor.RED
        robot_color = TokenColor.YELLOW
        offset = 0
    else:
        robot_color = TokenColor.RED
        user_color = TokenColor.YELLOW
        offset = 1

    alerted_robot_win = False

    def get_user_move():
        def move_is_valid(m: int):
            return board._column_heights[m - 1] < 6 
        move = -1
        print()
        solution = solver.get_solution(board.state)
        if solution.value > 0:
            fun_type(f" >>> you can (theoretically) win in {floor((45 - len(board.state))/2) - solution.value} move(s).")
        if solution.value == 0:
            fun_type(f" >>> you can (theoretically) force a draw.")
        if solution.value < 0:
            fun_type(f" >>> you will be defeated in {floor((44 - len(board.state))/2) + solution.value} move(s).")
        move = get_integer("Your turn! Enter desired column (1-7)", 1, 7)
        while not move_is_valid(move):
            fun_type("That column is full! Try again.")
            move = get_integer("Enter desired column (1-7)", 1, 7)

        def play_move():
            token_name, _ = drop_token_in_column(object_controller, user_color, move)
            time.sleep(0.5)
            attach_detach_helper.wait_for_token_to_fall(token_name, board._column_heights[move - 1] + 1, object_controller,
                                                        drop_token_callback, lodge_token_callback, play_move)
        
        previous_snap = video_camera.get_snapshot()
        play_move()
        time.sleep(1) # wait to take picture
        current_snap = video_camera.get_snapshot()
        camera_determined_move = get_column_played(board._column_heights, previous_snap, current_snap)
        fun_type(f"Camera processing complete - move in column {camera_determined_move} detected")

        if camera_determined_move != move:
            fun_type("My camera system is not working so well.")
            fun_type(f"It thought you played move {camera_determined_move}, but I know you actually played {move}.")
            fun_type("I'll cheat a bit and pretend I used my camera correctly.")
            # V--- HACK in case camera is wrong
            camera_determined_move = move

        board.update_with_char(str(camera_determined_move))

    def get_robot_move():
        nonlocal alerted_robot_win
        print()
        fun_type("The robot is making its move, please wait...")
        solution = solver.get_solution(board.state)
        if solution.value > 0:
            fun_type(f" >>> the robot will win in {floor((45 - len(board.state))/2) - solution.value} move(s).")
        if solution.value == 0:
            fun_type(f" >>> the robot can (theoretically) be forced into a draw.")
        if solution.value < 0:
            fun_type(f" >>> the robot can (theoretically) be defeated in {floor((44 - len(board.state))/2) + solution.value} move(s).")
        move = solution.best_move
        value = solution.value
        if value > 0 and not alerted_robot_win:
            fun_type("Looks like you have made a mistake - the robot is now guaranteed to win!")
            alerted_robot_win = True

        robot_play_token_in_column(robot, move, board._column_heights[move - 1] + 1, 
                                   robot_color, drop_token_callback, lodge_token_callback)
        
        board.update_with_char(str(move))
        
    counter = 0

    actions = [get_user_move, get_robot_move]

    while board.winner == None and counter < 42:
        actions[(counter + offset) % 2]()
        counter += 1

    if board.winner == user_color:
        fun_type("I'm not sure how this happened, but you managed to win. Congratulations!", delay=2)
    if board.winner == None:
        fun_type("You forced the robot into a draw. Congratulations!", delay=2)
    if board.winner == robot_color:
        fun_type("Looks like the robot won again. Better luck next time!", delay=2)

    # for column in range(7):
    #     robot_play_token_in_column(
    #         robot,
    #         column + 1,
    #         2,
    #         list(TokenColor)[column % 2], # alternate colors,
    #         drop_token_callback,
    #         lodge_token_callback
    #     )

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print()
