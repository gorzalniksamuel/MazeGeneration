using DataStructures: Queue, enqueue!, dequeue!

mutable struct Node
    x::Int # index in horizontal direction
    y::Int # index in vertical direction
    visited::Bool 
    is_wall::Bool # determines wether node can be part of a path 
    on_path::Union{Bool, Nothing}
end

struct MazeViz
    visualization::String
end

mutable struct Maze
    nodes::Matrix{Node}
    visual::Union{MazeViz, Nothing}
    path::Union{Vector{Node}, Nothing}
end

function neighbors(node::Node, grid::Array{Node, 2})
    # Finds the indirect neighbors (stepsize 2) of a node
    neighbors = []
    # we disregard diagonals / stepsize 2 to ensure that walls are placed in a useful manner
    # e.g. we ensure walls surrounding the maze completely if we start at (2,2) to construct the maze
    for (dx, dy) in ((-2, 0), (2, 0), (0, -2), (0, 2))
        dx, dy = node.x + dx, node.y + dy
        if dx > 0 && dy > 0 && dx <= size(grid, 1) && dy <= size(grid, 2)
            push!(neighbors, grid[dx, dy])
        end
    end
    return neighbors
end

function create_grid(height::Int, width::Int)
    grid = Array{Node, 2}(undef, height, width)
    # Iterate over the given grid size
    for x in 1:height
        for y in 1:width
            # Init all nodes as walls
            grid[x, y] = Node(x, y, false, true, nothing)
        end
    end
    return grid
end

function generate_maze!(grid::Array{Node, 2})
    # Generates a maze applying Randomized DFS
    @assert size(grid, 1) > 2 && size(grid, 2) > 2 # 2x2 maze not useful since we look for neighbors with stepsize 2

    stack = []
    # prevent cases where we dont have a boundary -> randomized start which is not on an edge 
    start_node = grid[2,2]                  # Alternative (border not necessarily a wall): grid[rand(2:(size(grid,1)-1)), rand(2:(size(grid,2))-1)]                                                   
    start_node.visited = true
    start_node.is_wall = false
    push!(stack, start_node)
    
    # DFS without recursion (basically BFS with a stack instead of a queue)
    while !isempty(stack)
        curr = stack[end]
        # get copy of neighbor nodes which only contains not visited
        unvisited_neighbors = filter(n -> !n.visited, neighbors(curr, grid)) #
        
        # if we have not visited neighbors
        if !isempty(unvisited_neighbors)
            # pick random neighbor which has not been visited yet (randomized DFS)
            next_node = rand(unvisited_neighbors)
            # change attributes in order to get valid path
            next_node.visited = true
            next_node.is_wall = false
            
            # remove wall between current and next_node (get indices and set is_wall to false)
            middle_x = div(curr.x + next_node.x, 2)
            middle_y = div(curr.y + next_node.y, 2)
            grid[middle_x, middle_y].is_wall = false
            
            push!(stack, next_node)
        else
            # if node has no valid neighbors (pop from stack)
            pop!(stack)
        end
    end
end
 
function pick_random_start_end(grid::Array{Node, 2})
    # picks valid start/end points at random (meaning they are on the grid and !is_wall)
    valid_nodes = filter(n -> !n.is_wall, grid)
    start = rand(valid_nodes)
    goal = rand(filter(n -> n != start, valid_nodes))
    return start, goal
end

#########################
# Solve right hand rule #
#########################
function solve(maze::Matrix{Node}, start::Node, goal::Node)
    # returns vector with nodes in correct order to find the path from start to end 
    current = start
    path = [start] # begin at start node 

    directions = [(0, 1), # go up  
                  (1, 0), # right 
                  (0, -1), # down 
                  (-1, 0)] # left
    dir_index = 1 # assume we start facing to the right 

    # loop until goal reached
    while current != goal 
        # Check direction to the right of current direction
        # By taking mod 4 we make sure that we will not be out of bounds
        right_dir = directions[mod1(dir_index + 1, 4)] # mod after floor (rotation to the right)
        
        # calculate new coordinates by adding dx/dy
        new_x = current.x + right_dir[1] 
        new_y = current.y + right_dir[2]
    
        # goto new coordinates 
        if is_valid_goal(maze, new_x, new_y)
            dir_index = mod1(dir_index + 1, 4) # go right
            current = maze[new_x, new_y]
        else
            # else try to move in forward direction
            forward_dir = directions[dir_index]
            new_x = current.x + forward_dir[1]
            new_y = current.y + forward_dir[2]

            if is_valid_goal(maze, new_x, new_y)
                current = maze[new_x, new_y]
            else
                # if forward durection is not possible -> move left
                dir_index = mod1(dir_index - 1, 4)
            end
        end
        push!(path, current)
    end

    return path
end

function is_valid_goal(maze::Matrix{Node}, x::Int, y::Int)
    return x > 0 && y > 0 && x <= size(maze, 1) && y <= size(maze, 2) && !maze[x, y].is_wall
end

##################
# solve with BFS #
##################
function solve_bfs(maze::Matrix{Node}, start::Node, goal::Node)
    # BFS for finding path from start to end
    queue = Queue{Tuple{Node, Vector{Node}}}()
    enqueue!(queue, (start, [start]))
    visited = Set([start])

    while !isempty(queue)
        # current node and path vector (so far)
        (current, path) = dequeue!(queue)

        # stopping criterion
        if current == goal
            return path
        end
        
        # get all valid neighbors and add them to the queue
        for neighbor in direct_neighbors(current, maze)
            if !neighbor.is_wall && !(neighbor in visited)
                # add tuple to queue consisting of neighbor and neighbor concatenated with current path-vector
                enqueue!(queue, (neighbor, vcat(path, [neighbor])))
                push!(visited, neighbor)
            end
        end
    end

    return [] # no path found
end

function direct_neighbors(node::Node, grid::Array{Node, 2})
    # Finding neighbors of given Node with stepsize 1
    neighbors = []
    for (dx, dy) in ((0, 1), (1, 0), (0, -1), (-1, 0))
        dx, dy = node.x + dx, node.y + dy
        if dx > 0 && dy > 0 && dx <= size(grid, 1) && dy <= size(grid, 2)
            push!(neighbors, grid[dx, dy])
        end
    end
    return neighbors
end

# Function to display the maze
function display_maze(grid::Array{Node, 2}, start::Node, goal::Node, mode="bfs") #schould receive path vector to fill with P
    if mode == "bfs"
        path = solve_bfs(grid, start, goal)
    else
        path = solve(grid, start, goal)
    end
    
    for p in path
        grid[p.x, p.y].on_path = true
    end 

    io = IOBuffer()
    for x in 1:size(grid, 1)
        for y in 1:size(grid, 2)
            node = grid[x, y]
            if node == start
                print(io, "S")
            elseif node == goal
                print(io, "E")
            elseif node.is_wall
                print(io, "█") 
            elseif node.on_path == true
                print(io, "X")
            else
                print(io, " ")
            end
        end
        println(io)
    end
    return String(take!(io))
end

function maze(height::Int, width::Int, mode)
    grid = create_grid(height, width)
    generate_maze!(grid)

    start, goal = pick_random_start_end(grid)

    # compute path / viz
    maze = Maze(grid, MazeViz(display_maze(grid, start, goal, mode)), nothing)
    return maze, start, goal
end 


function test_solve_functions()
    # Init test cases
    test_cases = [
        (21, 41, "bfs"),
        (21, 41, "rhr"),
        (15, 31, "bfs"),
        (15, 31, "rhr"),
        (11, 21, "bfs"),
        (11, 21, "rhr"),
        (9, 17, "bfs"),
        (9, 17, "rhr"),
        (5, 11, "bfs"),
        (5, 11, "rhr")
    ]
    
    test_results = []

    for (height, width, mode) in test_cases
        try
            test_maze, start, goal = maze(height, width, mode)
            if mode == "bfs"
                path = solve_bfs(test_maze.nodes, start, goal)
            else
                path = solve(test_maze.nodes, start, goal)
            end

            # Checking if the path is valid + reaches the goal
            is_valid = path[end] == goal
            test_results = vcat(test_results, (height, width, mode, is_valid))
        catch e
            test_results = vcat(test_results, (height, width, mode, false, e))
        end
    end

    return test_results
end

function main()
    # Display a sample maze
    test_maze, start, goal = maze(21, 41, "bfs")
    print(test_maze.visual.visualization)
    println("Path Using Right-Hand-Rule:")
    test_maze.path = solve(test_maze.nodes, start, goal)
    println(test_maze.path)
    println()
    println("Optimal path:")
    path = solve_bfs(test_maze.nodes, start, goal)
    println(path)
    
    println("Running test cases...")
    test_results = test_solve_functions()

    for result in test_results
        if length(result) == 4
            height, width, mode, is_valid = result
            if is_valid
                println("Test passed for maze size: $height x $width with mode: $mode")
            else
                println("Test failed for maze size: $height x $width with mode: $mode")
            end
        else
            height, width, mode, is_valid, e = result
            println("Test encountered an error for maze size: $height x $width with mode: $mode. Error: $e")
        end
    end
end

main()
