import tkinter as tk
from tkinter import ttk, filedialog
from models import RobotWorld, Position
from single_goal_search import BFS, DFS, AStar, GBFS, CUS1, CUS2
from multi_goals_search import MultiGoalBFS, MultiGoalDFS, MultiGoalGBFS, MultiGoalAStar, MultiGoalCUS1, MultiGoalCUS2
from typing import List, Set
import os     
import time       

            
class RobotNavGUI:
    """
    A GUI application for visualizing and solving robot navigation problems.
    
    This class provides a graphical interface for loading robot world problems,
    visualizing the world, and running various pathfinding algorithms to solve 
    single-goal and multi-goal navigation tasks.
    """
    
    MIN_CELL_SIZE = 30  # Minimum size of a cell
    MAX_CELL_SIZE = 50  # Maximum size of a cell
    
    # Dictionary mapping algorithms names to their respective classes
    ALGORITHMS = {
        "BFS": BFS,
        "DFS": DFS,
        "GBFS": GBFS,
        "A*": AStar,
        "CUS1": CUS1, # Bidirectional BFS
        "CUS2": CUS2, # Bidirectional A*
    }
    
    MULTI_GOAL_ALGORITHMS = {
        'Multi-BFS': MultiGoalBFS,
        'Multi-DFS': MultiGoalDFS,
        'Multi-GBFS': MultiGoalGBFS,
        'Multi-A*': MultiGoalAStar, 
        'Multi-CUS1': MultiGoalCUS1, # Multi Bidirectional BFS
        'Multi-CUS2': MultiGoalCUS2 # Multi Bidirectional A*
    }
    
    # Colors for visualizing multiple paths
    ARROW_COLORS = [
        "#00008B",  # Dark Blue
        "#8B0000",  # Dark Red
        "#006400",  # Dark Green
        "#8B4500",  # Dark Orange
        "#4A0E4E",  # Indigo
        "#556B2F",  # Dark Olive Green
        "#800080",  # Purple
        "#8B008B",  # Dark Magenta
    ]

    PATH_COLORS = [
        "#87CEFA",  # Light Sky Blue
        "#FFA07A",  # Light Salmon
        "#90EE90",  # Light Green
        "#FFD700",  # Gold
        "#B19CD9",  # Light Purple
        "#D3E878",  # Light Yellow Green
        "#DDA0DD",  # Plum
        "#FFB6C1",  # Light Pink
    ]
    
    def __init__(self, master):
        """
        Initialize the RobotBotNavGUI instance.

        Args:
            master: The root window of the application
        """
        self.master = master
        self.master.title("Assignment 1 - Robot Navigation")
        self.world = None   # Will hold the RobotWorld instance
        self.path = []      # Will hold the path for single-goal search
        self.visited_cells = set()    # Set of visited cells during search
        self.cell_size = self.MIN_CELL_SIZE     # Initial cell size
        self.multi_paths = []   # Will hold paths for multi-goal search
        
        self.path_visibility = []  # List to store visibility state of each path
        self.path_checkbuttons = []  # List to store checkbuttons for each path

        
        self.setup_ui()
        
    def setup_ui(self):
        """Set up the main UI components."""
        # Make the window resizable
        self.master.resizable(True, True)
        self.master.grid_columnconfigure(0, weight=1)
        self.master.grid_columnconfigure(1, weight=3)
        self.master.grid_rowconfigure(0, weight=1)

        # Create left frame and respective UI components
        self.left_frame = ttk.Frame(self.master, padding='10')
        self.left_frame.grid(row=0, column=0, sticky='nsew')
        self.left_frame.grid_rowconfigure(3, weight=1)
        self.left_frame.grid_columnconfigure(0, weight=1)

        self.create_title()
        self.create_load_section()
        self.create_algorithm_section()
        self.create_output_text()

        # Create right frame and respective UI components
        self.right_frame = ttk.Frame(self.master, padding='10')
        self.right_frame.grid(row=0, column=1, sticky='nsew')
        self.right_frame.grid_rowconfigure(0, weight=1)
        self.right_frame.grid_columnconfigure(0, weight=1)

        self.create_canvas()
        self.create_zoom_controls()
        self.create_path_visibility_frame()
    
    def create_path_visibility_frame(self):
        """Create a frame with checkboxes to toggle path visibility."""
        self.visibility_frame = ttk.LabelFrame(self.left_frame, text='Path Visibility')
        self.visibility_frame.grid(row=4, column=0, sticky='ew', pady=10)
        self.visibility_frame.grid_columnconfigure(0, weight=1)
    
    def update_path_visibility_controls(self):
        """Update the path visibility controls based on the current paths."""
        # Clear existing checkbuttons
        for cb in self.path_checkbuttons:
            cb.destroy()
        self.path_checkbuttons.clear()

        # Create new checkbuttons for each path
        self.path_visibility = [tk.BooleanVar(value=True) for _ in self.multi_paths]
        for i, var in enumerate(self.path_visibility):
            cb = ttk.Checkbutton(
                self.visibility_frame, 
                text=f'Path {i+1} - Goal {i+1}',
                variable=var,
                command=self.draw_world_multi
            )
            cb.grid(row=i, column=0, sticky='w')
            self.path_checkbuttons.append(cb)

        
    def create_title(self):
        """Create and place the title label"""
        title_label = ttk.Label(self.left_frame, text='Robot Navigation Problem', font=('', 22, 'bold'))
        title_label.grid(row=0, column=0, pady=20)
        
    def create_load_section(self):
        """Create and place the problem loading section"""
        load_frame = ttk.Frame(self.left_frame)
        load_frame.grid(row=1, column=0, sticky='nsew')
        load_frame.grid_columnconfigure(0, weight=1)
        
        ttk.Button(load_frame, text='Load Problem', command=self.load_problem).pack(pady=15)
        ttk.Label(load_frame, text='You must load a problem file to begin', font=('', 12,'')).pack()
        
        self.info_label = ttk.Label(load_frame, text='', font=('', 11, 'italic'))
        self.info_label.pack(pady=15)
        
    def create_algorithm_section(self):
        """Create and place the algorithm selection section."""
        algo_frame = ttk.Frame(self.left_frame)
        algo_frame.grid(row=2, column=0, sticky='ew')
        algo_frame.grid_columnconfigure(0, weight=1)
        algo_frame.grid_columnconfigure(1, weight=1)
        
        self.create_single_goal_frame(algo_frame)
        self.create_multi_goal_frame(algo_frame)
        
    def create_single_goal_frame(self, parent):
        """Create the frame for single-goal algorithm selection."""
        single_goal_frame = ttk.LabelFrame(parent, text='Single Goal Search')
        single_goal_frame.grid(row=0, column=0, padx=10, sticky='ew')
        
        self.algorithm_var = tk.StringVar(value='BFS')
        ttk.OptionMenu(single_goal_frame, self.algorithm_var, 'BFS', *self.ALGORITHMS.keys()).pack(pady=5)
        ttk.Button(single_goal_frame, text='Solve', command=self.solve_single).pack(pady=5)
        
    def create_multi_goal_frame(self, parent):
        """Create the frame for multi-goal algorithm selection."""
        multi_goal_frame = ttk.LabelFrame(parent, text='Multi Goal Search')
        multi_goal_frame.grid(row=0, column=1, padx=10, sticky='ew')
        
        self.multiple_algorithm_var = tk.StringVar(value='Multi-BFS')
        ttk.OptionMenu(multi_goal_frame, self.multiple_algorithm_var, 'Multi-BFS', *self.MULTI_GOAL_ALGORITHMS.keys()).pack(pady=5)
        ttk.Button(multi_goal_frame, text='Solve All Goals', command=self.solve_multi).pack(pady=5)
        
    def create_canvas(self):
        """Create and place the main canvas for world visualization."""
        self.canvas_frame = ttk.Frame(self.right_frame)
        self.canvas_frame.grid(row=0, column=0, sticky='nsew')
        self.canvas_frame.grid_rowconfigure(0, weight=1)
        self.canvas_frame.grid_columnconfigure(0, weight=1)
        
        self.canvas = tk.Canvas(self.canvas_frame)
        self.canvas.grid(row=0, column=0, sticky='nsew')
        
    def create_zoom_controls(self):
        """Create and place zoom control buttons."""
        zoom_frame = ttk.Frame(self.right_frame)
        zoom_frame.grid(row=1, column=0, sticky='ew', pady=10)
        ttk.Label(zoom_frame, text="Zoom:").pack(side=tk.LEFT)
        
        style = ttk.Style()
        style.configure("Big.TButton", font=("", 16))  # "" for default font, 16 for size

        ttk.Button(zoom_frame, text="-", command=self.zoom_out, style="Big.TButton").pack(side=tk.LEFT)
        ttk.Button(zoom_frame, text="+", command=self.zoom_in, style="Big.TButton").pack(side=tk.LEFT)
        
    def create_output_text(self):
        """Create and place the output text area."""
        self.output_text = tk.Text(self.left_frame, height=10, wrap=tk.WORD)
        self.output_text.grid(row=3, column=0, sticky='nsew', pady=20)
        
    def load_problem(self):
        """Load a problem file and initialize the world."""
        self.filename = filedialog.askopenfilename(filetypes=[("Text files", "*.txt")]) # Only accept .txt file
        
        if self.filename:
            self.world = RobotWorld.from_file(self.filename)
            self.clear_search()
            self.calculate_cell_size()
            self.update_canvas_size()
            self.draw_world_multi()
            self.info_label.config(text=f'Loaded problem from {self.filename}')


    def calculate_cell_size(self):
        """Calculate the appropriate cell size based on the window and world dimensions."""
        if self.world:
            # Get the available space for the canvas
            available_width = self.right_frame.winfo_width() - 20  # Subtract padding
            available_height = self.right_frame.winfo_height() - 40  # Subtract space for zoom controls and padding
            
            # Calculate the cell size that would fit the world in the available space
            width_based_size = available_width / self.world.cols
            height_based_size = available_height / self.world.rows
            
            # Use the smaller of the two sizes to ensure the world fits
            calculated_size = min(width_based_size, height_based_size)
            
            # Clamp the calculated size between MIN_CELL_SIZE and MAX_CELL_SIZE
            self.cell_size = max(min(calculated_size, self.MAX_CELL_SIZE), self.MIN_CELL_SIZE)

    def update_canvas_size(self):
        """Update the canvas size based on the world dimensions and cell size."""
        if self.world:
            canvas_width = self.world.cols * self.cell_size + 5
            canvas_height = self.world.rows * self.cell_size + 5
            self.canvas.config(width=canvas_width, height=canvas_height, scrollregion=(0, 0, canvas_width, canvas_height))

            
    def clear_search(self):
        """Clear all search-related data and redraw the world"""
        self.path = []
        self.multi_paths = []
        self.visited_cells = set()
        if self.world:
            self.draw_world_multi()
        self.info_label.config(text='')
        self.output_text.delete('1.0', tk.END)
    
    
    def zoom_in(self):
        """Increase the cell size for zooming in, within defined limits."""
        if self.cell_size < self.MAX_CELL_SIZE:
            self.cell_size = min(self.cell_size * 1.2, self.MAX_CELL_SIZE)
            self.update_canvas_size()
            self.draw_world_multi()

    def zoom_out(self):
        """Decrease the cell size for zooming out, within defined limits."""
        if self.cell_size > self.MIN_CELL_SIZE:
            self.cell_size = max(self.cell_size / 1.2, self.MIN_CELL_SIZE)
            self.update_canvas_size()
            self.draw_world_multi()
    
    
    def create_visualizing_algorithm(self, algorithm_class):
        """
        Create a visualizing version of the given algorithm class.
        
        This method creates a subclass of the given algorithm that updates the GUI
        as it searches, allowing for real-time visualization of the search process.
        
        Args:
            algorithm_class (class): The original searching algorithm class to be visualized.
            
        Returns:
            class: A new subclass of the original algorithm with added visualization capabilities.
            
        Details:
            - The returned class overrides the 'visit_node' method to update the GUI.
            - It adds each visited position to the GUI's set of visited cells.
            - The world is redrawn after each node visit to show search progress.
            - A small delay is added between node visits for better visualization.
        """
        class VisualizingAlgorithm(algorithm_class):
            def __init__(self, world, gui):
                super().__init__(world)
                self.gui = gui
            
            def visit_node(self, position):
                # Add the visited position to the set of visited cells
                self.gui.visited_cells.add(position.as_tuple())
                # Redraw the world to show the newly visited cell
                self.gui.draw_world_multi()
                # Update the GUI to reflect changes
                self.gui.master.update()
                # Pause briefly to show the search progress
                time.sleep(0.1)
        
        return VisualizingAlgorithm(self.world, self)
    

    #__________________________________________________________________________        
    # Single goal search 
    def solve_single(self):
        """
        Solve the single-goal problem using the selected algorithm
        
        This method initiates the searching process for a single goal, visualized 
        the search, and displaying the results
        """
        if not self.world:
            self.info_label.config(text="Please load a problem first")  # Send an announcement if the problem is not loaded
            return
        
        self.clear_search()
        algorithm_class = self.ALGORITHMS[self.algorithm_var.get()]
        visualizing_algorithm = self.create_visualizing_algorithm(algorithm_class)
        self.path, nodes_created = visualizing_algorithm.find_path()
        
        self.draw_world()
        self.display_single_result(self.algorithm_var.get(), nodes_created)
        
    
    def draw_world(self):
        """
        Draw the world for single-goal visualization.
        
        This method renders the entire grid world on the canvas, including walls,
        visited cells, the path (if found), start position, and goal positions.
        
        Details: 
            - Grid cells are colored as follows:
                * White: Unvisited, passable cell
                * Gray: wall
                * Light blue (#C8C8E8): Visited cell
                * Yellow: Cell on the path found
            - The method uses self.cell_size to determine the size of each grid cell.
            - Arrows are drawn to show the direction of the path.
        """
        self.canvas.delete("all")
        self.update_canvas_size()

        # Draw grid and cells
        for row in range(self.world.rows):
            for col in range(self.world.cols):
                x1 = col * self.cell_size
                y1 = row * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                # Default white cell with black outline
                fill_color = 'white'
                outliner_color = 'black'
                
                # Check for wall
                pos = Position(col, row)
                if any(wall.contains(pos) for wall in self.world.walls):
                    fill_color = 'gray' 
                # Check for visited cell
                elif (col, row) in self.visited_cells:
                    fill_color = '#C8C8E8' 
                # Check for path
                if self.path and Position(col, row) in self.path:
                    fill_color = 'yellow'
                
                self.canvas.create_rectangle(x1, y1, x2, y2, 
                                           fill=fill_color, outline=outliner_color)
        
       # Draw start position (red rectangle)
        start_x = self.world.start.x * self.cell_size
        start_y = self.world.start.y * self.cell_size
        self.canvas.create_rectangle(start_x, start_y, start_x + self.cell_size, start_y + self.cell_size, 
                                    fill='red', outline='black')
        
        # Draw goal positions (green rectangles)
        for goal in self.world.goals:
            goal_x = goal.x * self.cell_size
            goal_y = goal.y * self.cell_size
            self.canvas.create_rectangle(goal_x, goal_y, goal_x + self.cell_size, goal_y + self.cell_size, 
                                        fill='green', outline='black')
            
        # Draw arrows for the path
        if self.path:
            for i in range(len(self.path) - 1):
                current = self.path[i]
                next_pos = self.path[i+1]
                self.draw_arrow_single(current.x, current.y, next_pos.x, next_pos.y)
        
        
    
    def draw_arrow_single(self, x1, y1, x2, y2, color='black'):
        """Draw an arrow for the single-goal path."""
        # Calculate midpoints for cell centers
        mx1 = x1 * self.cell_size + self.cell_size / 2
        my1 = y1 * self.cell_size + self.cell_size / 2
        mx2 = x2 * self.cell_size + self.cell_size / 2
        my2 = y2 * self.cell_size + self.cell_size / 2
        
        # Draw arrow line
        self.canvas.create_line(mx1, my1, mx2, my2, fill=color, width=2, arrow=tk.LAST)
        
        
    def display_single_result(self, method, nodes_created):
        """
        Display the result of a single-goal search algorithm.

        Args:
            method (str): The name of the search algorithm used.
            nodes_created (int): The number of nodes created during the search.
        """
        self.output_text.delete('1.0', tk.END)
        filename = os.path.basename(self.filename)
        if self.path:
            goal = self.path[-1]
            path_dirs = self.get_path_directions(self.path)
            self.output_text.insert(tk.END, f'{filename} {method}\n<Node ({goal.x}, {goal.y})> {nodes_created}\n')
            self.output_text.insert(tk.END, f'{path_dirs}\n')
        else:
            self.output_text.insert(tk.END, f'{filename} {method}\nNo goal is reachable; {nodes_created}\n')
            

    #__________________________________________________________________________
    # Multi goal world drawing       
    def solve_multi(self):
        """Solve the multi-goal search problem and update the display."""
        if not self.world:
            self.info_label.config(text='Please load a problem first')
            return
        
        self.clear_search()
        algorithm_name = self.multiple_algorithm_var.get()
        algorithm_class = self.MULTI_GOAL_ALGORITHMS[algorithm_name]
        visualizing_algorithm = self.create_visualizing_algorithm(algorithm_class)
        self.multi_paths, total_nodes = visualizing_algorithm.find_all_goals()
        
        self.update_path_visibility_controls()  # Update visibility controls
        self.draw_world_multi()
        self.display_multi_result(algorithm_name, total_nodes)
        
    
    def draw_world_multi(self):
        """Draw the world for multi-goal searching, including grid, paths, and markers."""
        self.canvas.delete("all")
        self.update_canvas_size()
        
        # Draw grid and basic elements first
        for row in range(self.world.rows):
            for col in range(self.world.cols):
                x1 = col * self.cell_size
                y1 = row * self.cell_size
                x2 = x1 + self.cell_size
                y2 = y1 + self.cell_size
                
                fill_color = 'white'
                outline_color = 'black'
                
                pos = Position(col, row)
                if any(wall.contains(pos) for wall in self.world.walls):
                    fill_color = 'gray'
                elif (col, row) in self.visited_cells:
                    fill_color = '#C8C8E8'
                
                self.canvas.create_rectangle(x1, y1, x2, y2, 
                                        fill=fill_color, outline=outline_color)
        
        
        # Collect and draw colored cells for visible paths
        cell_colors = [[set() for _ in range(self.world.cols)] for _ in range(self.world.rows)]
        for i, path in enumerate(self.multi_paths):
            if self.path_visibility[i].get():  # Check if path is visible
                color = self.PATH_COLORS[i % len(self.PATH_COLORS)]
                for pos in path:
                    cell_colors[pos.y][pos.x].add(color)
        
        # Color the cells based on paths
        for row in range(self.world.rows):
            for col in range(self.world.cols):
                colors = cell_colors[row][col]
                if colors:
                    x1 = col * self.cell_size
                    y1 = row * self.cell_size
                    x2 = x1 + self.cell_size
                    y2 = y1 + self.cell_size
                    
                    if len(colors) == 1:
                        color = list(colors)[0]
                        self.canvas.create_rectangle(x1, y1, x2, y2, 
                                                    fill=color, stipple='gray50', outline='black')
                    elif len(colors) > 1:
                        self.draw_diagonal_cell(x1, y1, x2, y2, list(colors)[:2])
        
        # Draw start position
        start_x = self.world.start.x * self.cell_size
        start_y = self.world.start.y * self.cell_size
        self.canvas.create_rectangle(start_x, start_y, 
                                    start_x + self.cell_size, 
                                    start_y + self.cell_size, 
                                    fill='red', outline='black')
        
        
        # Create a dictionary to store the order of reached goals
        goal_order = {path[-1]: i+1 for i, path in enumerate(self.multi_paths)}
        
        # Draw goal positions with numbered markers
        for goal in self.world.goals:
            goal_x = goal.x * self.cell_size
            goal_y = goal.y * self.cell_size
            self.canvas.create_rectangle(goal_x, goal_y, 
                                        goal_x + self.cell_size, 
                                        goal_y + self.cell_size, 
                                        fill='green', outline='black')
            
            # Add numbered marker for each goal if it was reached
            if goal in goal_order:
                marker_size = min(self.cell_size // 3, 15)  # Adjust size based on cell size
                
                # Position the marker at the top-right corner of the goal cell
                marker_x = goal_x + self.cell_size - marker_size // 4  # Shift a little to the left for alignment
                marker_y = goal_y + self.cell_size - marker_size // 4  # Shift up a bit for alignment
                
                # Draw yellow circle for the marker background
                self.canvas.create_oval(marker_x - marker_size/2, marker_y - marker_size/2,
                                        marker_x + marker_size/2, marker_y + marker_size/2,
                                        fill='yellow', outline='black')
                
                # Add the number to the marker
                self.canvas.create_text(marker_x, marker_y, text=str(goal_order[goal]), 
                                        font=('', max(int(marker_size * 0.6), 10), 'bold'), fill='black')
        
       
        # Draw arrows for each visible path 
        num_paths = len(self.multi_paths)
        for i, path in enumerate(self.multi_paths):
            if self.path_visibility[i].get():  # Check if path is visible
                color = self.PATH_COLORS[i % len(self.PATH_COLORS)]
                arrow_color = self.ARROW_COLORS[i % len(self.ARROW_COLORS)]
                
                # Calculate offset based on path index, ensuring more spread
                offset = (i - (num_paths - 1) / 2) * (self.cell_size / (num_paths + 1))
                
                for j in range(len(path) - 1):
                    current = path[j]
                    next_pos = path[j+1]
                    self.draw_arrow_multi(current.x, current.y, next_pos.x, next_pos.y, arrow_color, offset, i)

        # Add clearer start/end markers for each path
        for i, path in enumerate(self.multi_paths):
            if path:
                # Start marker
                start = path[0]
                sx = start.x * self.cell_size + self.cell_size / 2
                sy = start.y * self.cell_size + self.cell_size / 2
                color = self.ARROW_COLORS[i % len(self.PATH_COLORS)]
                
                # Larger, more visible start marker
                start_radius = 3
                self.canvas.create_oval(
                    sx - start_radius, sy - start_radius,
                    sx + start_radius, sy + start_radius,
                    fill=color, outline=color, width=3
                )
        
        
        
    def draw_arrow_multi(self, x1, y1, x2, y2, color, offset, path_index):
        """
        Draw an arrow between cells with larger offset for clearer path separation.
        
        Args:
            x1, y1: Start cell coordinates
            x2, y2: End cell coordinates
            color: Arrow color
            offset: Pixel offset for the arrow (increased for better visibility)
        """
        
        # Calculate midpoints for cell centers
        mx1 = x1 * self.cell_size + self.cell_size / 2
        my1 = y1 * self.cell_size + self.cell_size / 2
        mx2 = x2 * self.cell_size + self.cell_size / 2
        my2 = y2 * self.cell_size + self.cell_size / 2
        
        # Determine if this is a diagonal move
        is_diagonal = x1 != x2 and y1 != y2
        
        # Define arrow properties
        arrow_width = max(2, self.cell_size // 20)
        arrow_head_length = max(6, self.cell_size // 6)
        
        # Apply offset
        if is_diagonal:
            dx = mx2 - mx1
            dy = my2 - my1
            length = (dx**2 + dy**2)**0.5
            nx = -dy / length
            ny = dx / length
            mx1 += nx * offset
            my1 += ny * offset
            mx2 += nx * offset
            my2 += ny * offset
        else:
            if x1 == x2:  # Vertical move
                mx1 += offset
                mx2 += offset
            else:  # Horizontal move
                my1 += offset
                my2 += offset
        
        # Draw arrow line
        self.canvas.create_line(
            mx1, my1, mx2, my2,
            fill=color, width=arrow_width, arrow=tk.LAST,
            arrowshape=(arrow_head_length, arrow_head_length + 5, arrow_head_length/2),
            capstyle=tk.ROUND, joinstyle=tk.ROUND
        )
         
    
    def draw_diagonal_cell(self, x1, y1, x2, y2, colors):
        """Draw a cell with diagonal split for overlapping cells.

        Args:
            x1, y1, x2, y2 (int): Cell coordinates
            colors (list): Two colors for the diagonal split
        """
        # Draw two triangles in different colors
        self.canvas.create_polygon(x1, y1, x2, y1, x2, y2, 
                                   fill=colors[0], outline='black')
        self.canvas.create_polygon(x1, y1, x1, y2, x2, y2, 
                                   fill=colors[1], outline='black')
    
    
    
    def display_multi_result(self, method, total_nodes):
        """
        Display the result of the multi-goal searching.

        Args:
            method (str): The name of the search algorithm used
            total_nodes (int): Total number of nodes created during the search
        """
        # Display the result of the multi-threaded search
        self.output_text.delete('1.0', tk.END)
        filename = os.path.basename(self.filename)
        if self.multi_paths:
            self.output_text.insert(tk.END, f'{filename} {method}\n')
            self.output_text.insert(tk.END, f'Found {len(self.multi_paths)} paths! Nodes created: {total_nodes}\n')
            
            for i, path in enumerate(self.multi_paths, 1):
                goal = path[-1]
                path_dirs = self.get_path_directions(path)
                self.output_text.insert(tk.END, f'\nPath {i} to Goal {i}: <Node ({goal.x}, {goal.y})>\n')
                self.output_text.insert(tk.END, f'{path_dirs}\n')
        else:
            self.output_text.insert(tk.END, f'{filename} {method}\nNo goals are reachable; {total_nodes}\n')
            

    #__________________________________________________________
    # Path directions
    def get_path_directions(self, path):
        """
        Convert a path to a list of directional instructions.

        Args:
            path (list): List of Position objects respresenting the path

        Returns:
            str: A string representation of directional instructions
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
                