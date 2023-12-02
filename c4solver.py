#!/usr/bin/python3

"""
NOTE: board states are inputted as a continuous string indicating each 
sequential move.

So if player one plays column 4, then player two plays column 3, the board state 
is "43".

"""

import subprocess
import numpy as np
from dataclasses import dataclass

@dataclass
class Connect4Solution:
    best_move: int
    value: int
    player: int

class Connect4Solver:
    
    def __init__(self) -> None:
        self.connect4solver = subprocess.Popen(["./c4solver", "-a"], stdin=subprocess.PIPE, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE
        )
        # print("Loading solver...")

        # ignore the "Loaded ..." etc. line
        self.connect4solver.stderr.readline()

        # print("Ready!")
    

    def get_solution(self, board_state: str):

        self.connect4solver.stdin.write((board_state + "\n").encode())
        self.connect4solver.stdin.flush()
        output = self.connect4solver.stdout.readline().decode()

        moves = [int(v) for v in output.split(" ")[1:]]

        best_move = 1 + np.argmax(np.array(moves))
        player_num = 1 + len(output.split(" ")[0]) % 2
        return Connect4Solution(best_move, max(moves), player_num)
