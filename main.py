import sys
from typing import List
from models import RobotWorld, Position
from single_goal_search import BFS, DFS, AStar, GBFS, CUS1, CUS2
from multi_goals_search import MultiGoalBFS, MultiGoalDFS, MultiGoalGBFS, MultiGoalAStar, MultiGoalCUS1, MultiGoalCUS2
from gui import RobotNavGUI
import tkinter as tk


def run_cli(filename: str, method: str):
    """
    Run the robot navigation algorithm in CLI mode.

    Args:
        filename (str): The name of the file containing the world description.
        method (str): The name of the algorithm to use for searching.
    
    This function loads the world from a file, runs the specified algorithm,
    and prints the results to the console.
    """
    
    # Load the world from the specified file
    world = RobotWorld.from_file(filename)
    
    if world is None:
        print(f"No goal is reachable; 0")
        return
    
    # Define dictionaries for single-goal and multi-goal algorithms
    single_goal_algorithms = {
        'bfs': BFS,
        'dfs': DFS,
        'astar': AStar,
        'gbfs': GBFS,
        'cus1': CUS1, # Bidirectional BFS
        'cus2': CUS2, # Bidirectional A*
    }
    
    multi_goal_algorithms = {
        'multi-bfs': MultiGoalBFS,
        'multi-dfs': MultiGoalDFS,
        'multi-astar': MultiGoalAStar,
        'multi-gbfs': MultiGoalGBFS,
        'multi-cus1': MultiGoalCUS1, # Multi Bidirectional BFS
        'multi-cus2': MultiGoalCUS2 # Multi Bidirectional A*
    }
    
    
    print(f'{filename} {method.upper()}')
    
    # Determine if it's a multi-goal or single-goal method
    if method.startswith('multi-'):
        if method not in multi_goal_algorithms:
            print(f'Method {method.upper()} not recognized. Available multi-goal methods: {", ".join(multi_goal_algorithms.keys())}')
            return
        
        # Run the multi-goal algorithm
        algorithm = multi_goal_algorithms[method](world)
        paths, nodes_created = algorithm.find_all_goals()
        
        print(f'Found {len(paths)} paths! Nodes created: {nodes_created}')
        
        if paths:
           for i, path in enumerate(paths, 1):
               goal = path[-1]
               print(f'Path{i} to Goal {i}: <Node ({goal.x}, {goal.y})>')
               print(path_to_directions(path))
        else:
            print(f'No goals are reachable; {nodes_created}')

    else:
        if method not in single_goal_algorithms:
            print(f'Method {method} not recognized. Available single-goal methods: {", ".join(single_goal_algorithms.keys())}')
            return
        
        
        # Run the single-goal algorithm
        algorithm = single_goal_algorithms[method](world)
        path, nodes_created = algorithm.find_path()
        
        
        if path:
            goal = path[-1]
            print(f'<Node ({goal.x}, {goal.y})> {nodes_created}')
            print(path_to_directions(path))
            
        else:
            print(f'No goal is reachable; {nodes_created}')

def path_to_directions(path: List[Position]):
    """
    Convert a path of Positions to a string of directions.
    
    Args:
        path (List[Position]): A list of Position objects representing the path.
    
    Returns:
        str: A string representation of the directions to follow the path
    """
    directions = []
    for i in range(1, len(path)):
        current, next_pos = path[i-1], path[i]
        dx, dy = next_pos.x - current.x, next_pos.y - current.y
        
        if dx == 1:
            directions.append('right')
        elif dx == -1:
            directions.append('left')
        elif dy == 1:
            directions.append('down')
        elif dy == -1:
            directions.append('up')
    return str(directions)


def run_gui():
    """
    Run the robot navigation program in GUI mode.
    
    This function created a Tkinter root window and initializes the RobotNavGUI application.
    """
    root = tk.Tk()
    app = RobotNavGUI(root)
    root.mainloop()

    
def main():
    """
    Main function to run the robot navigation program.
    
    This function determines whether to run in CLI or GUI mode based on the command-line arguments.
    """
    if len(sys.argv) == 3:
        # CLI mode
        filename = sys.argv[1]
        method = sys.argv[2].lower()  # Convert to lowercase for consistency
        run_cli(filename, method)
    elif len(sys.argv) == 1:
        # GUI mode
        run_gui()
    else:
        print("Usage:")
        print('For CLI mode: python main.py <filename> <method>')
        print('For GUI mode: python main.py')


if __name__ == "__main__":
    main()
