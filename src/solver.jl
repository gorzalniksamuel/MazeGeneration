mutable struct Node
    x::Int # index in horizontal direction
    y::Int # index in vertical direction
    visited::Bool 
    is_wall::Bool # determines whether node can be part of a path 
end


function neighbors(node::Node, grid::Array{Node, 2})
    # Finds the indirect neighbors of a node
    neighbors = []
    # we disregard diagonals / stepsize 2 to ensure that walls are placed in a useful manner
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
    stack = []
    # prevent cases where we dont have a boundary -> start at cell 2,2
    start_node = grid[2, 2] 
    start_node.visited = true
    start_node.is_wall = false
    push!(stack, start_node)
    
    while !isempty(stack)
        curr = stack[end]
        unvisited_neighbors = filter(n -> !n.visited, neighbors(curr, grid))
        
        if !isempty(unvisited_neighbors)
            next_node = rand(unvisited_neighbors)
            next_node.visited = true
            next_node.is_wall = false
            
            # remove wall between current and next_node
            middle_x = div(curr.x + next_node.x, 2)
            middle_y = div(curr.y + next_node.y, 2)
            grid[middle_x, middle_y].is_wall = false
            
            push!(stack, next_node)
        else
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
function display_maze(grid::Array{Node, 2}, start::Node, goal::Node)
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


function solve(maze::Matrix{Node})
    # returns vector with nodes in correct order to find the path from start to end 
    return 
end 

struct MazeViz
    visualization::String
end

struct Maze
    nodes::Matrix{Node}
    visual::Union{MazeViz, Nothing}
    path::Union{Vector{Node}, Nothing}
end

function maze(height::Int, width::Int)
    grid = create_grid(height, width)
    generate_maze!(grid)

    start, goal = pick_random_start_end(grid)
    

    # compute path / viz
    maze = Maze(grid, MazeViz(display_maze(grid, start, goal)), nothing)

    
    return maze
end 


# Main function to create and display the maze
function main()
    test_maze = maze(21,21)
    print(test_maze.visual.visualization)
end

# Run the main function
main()