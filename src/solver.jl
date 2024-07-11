using DataStructures: Queue, enqueue!, dequeue!

mutable struct Node
    x::Int # index in horizontal direction
    y::Int # index in vertical direction
    visited::Bool 
    is_wall::Bool # determines whether node can be part of a path 
end

struct MazeViz
    visualization::String
end

struct Maze
    nodes::Matrix{Node}
    visual::Union{MazeViz, Nothing}
    path::Union{Vector{Node}, Nothing}
end

function neighbors(node::Node, grid::Array{Node, 2})
    # Finds the indirect neighbors of a node
    neighbors = []
    # we disregard diagonals / stepsize 2 to ensure that walls are placed in a useful manner
    # e.g. we ensure walls surrounding the maze completely if we start at (2,2) to construct the maze
    for (dx, dy) in ((-2, 0), (2, 0), (0, -2), (0, 2))
        nx, ny = node.x + dx, node.y + dy
        if nx > 0 && ny > 0 && nx <= size(grid, 1) && ny <= size(grid, 2)
            push!(neighbors, grid[nx, ny])
        end
    end
    return neighbors
end

function create_grid(height::Int, width::Int)
    grid = Array{Node, 2}(undef, height, width)
    # iterate over the given grid size
    for x in 1:height
        for y in 1:width
            # we initialize all nodes as walls
            grid[x, y] = Node(x, y, false, true)  
        end
    end
    return grid
end

# Function to generate a maze using randomized depth-first search
function generate_maze!(grid::Array{Node, 2})
    @assert size(grid, 1) > 2 && size(grid, 2) > 2

    stack = []
    # prevent cases where we dont have a boundary -> randomized start which is not on an edge 
    start_node = grid[2,2]                  #TODO: grid[rand(2:(size(grid,1)-1)), rand(2:(size(grid,2))-1)]                                                  
    start_node.visited = true
    start_node.is_wall = false
    push!(stack, start_node)
    

    # implementation of DFS without recursion (basically BFS with a stack instead of a queue)
    while !isempty(stack)
        curr = stack[end]
        # get copy of neighbor nodes which only contains not visited
        unvisited_neighbors = filter(n -> !n.visited, neighbors(curr, grid)) 
        
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
    # picks random start/end points which are valid (meaning they are on the grid and !is_wall)
    valid_nodes = filter(n -> !n.is_wall, grid)
    start = rand(valid_nodes)
    goal = rand(filter(n -> n != start, valid_nodes))
    return start, goal
end

# Function to display the maze
function display_maze(grid::Array{Node, 2}, start::Node, goal::Node) #schould receive path vector to fill with P
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
            else
                print(io, " ")
            end
        end
        println(io)
    end
    return String(take!(io))
end

#########################
# Solve right hand rule #
#########################
function solve(maze::Matrix{Node}, start::Node, goal::Node)
    # returns vector with nodes in correct order to find the path from start to end 
    current = start
    path = [start]

    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    dir_index = 1 # Start by facing right (0, 1)

    while current != goal
        # Check the direction to the right of the current direction
        right_dir = directions[mod1(dir_index + 1, 4)]
        new_x = current.x + right_dir[1]
        new_y = current.y + right_dir[2]

        if can_move(maze, new_x, new_y)
            dir_index = mod1(dir_index + 1, 4) # Turn right
            current = maze[new_x, new_y]
        else
            # Otherwise, try to move forward
            forward_dir = directions[dir_index]
            new_x = current.x + forward_dir[1]
            new_y = current.y + forward_dir[2]

            if can_move(maze, new_x, new_y)
                current = maze[new_x, new_y]
            else
                # If you can't move forward, turn left
                dir_index = mod1(dir_index - 1, 4)
            end
        end
        push!(path, current)
    end

    return path
end

function can_move(maze::Matrix{Node}, x::Int, y::Int)
    return x > 0 && y > 0 && x <= size(maze, 1) && y <= size(maze, 2) && !maze[x, y].is_wall
end

##################
# solve with BFS #
##################
function solve_bfs(maze::Matrix{Node}, start::Node, goal::Node)
    # BFS for finding an optimal path from start to end
    queue = Queue{Tuple{Node, Vector{Node}}}()
    enqueue!(queue, (start, [start]))
    visited = Set([start])

    while !isempty(queue)
        (current, path) = dequeue!(queue)

        if current == goal
            return path
        end

        for neighbor in direct_neighbors(current, maze)
            if !neighbor.is_wall && !(neighbor in visited)
                enqueue!(queue, (neighbor, vcat(path, [neighbor])))
                push!(visited, neighbor)
            end
        end
    end

    return []  # Return an empty path if no path is found
end

function direct_neighbors(node::Node, grid::Array{Node, 2})
    neighbors = []
    for (dx, dy) in ((0, 1), (1, 0), (0, -1), (-1, 0))
        nx, ny = node.x + dx, node.y + dy
        if nx > 0 && ny > 0 && nx <= size(grid, 1) && ny <= size(grid, 2)
            push!(neighbors, grid[nx, ny])
        end
    end
    return neighbors
end








function maze(height::Int, width::Int)
    grid = create_grid(height, width)
    generate_maze!(grid)

    start, goal = pick_random_start_end(grid)
    

    # compute path / viz
    maze = Maze(grid, MazeViz(display_maze(grid, start, goal)), nothing)

    
    return maze, start, goal
end 


# Main function to create and display the maze
function main()
    test_maze, start, goal = maze(21,21)
    print(test_maze.visual.visualization)
    path = solve(test_maze.nodes, start, goal)
    println("Path Using Right-Hand-Rule:")
    println(path)
    println()
    println("Optimal path:")
    path = solve_bfs(test_maze.nodes, start, goal)
    println(path)
end

# Run the main function
main()