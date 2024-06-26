#=
module MazeGeneration

export Node, neighbors, Maze, generate_maze!, MazeGeneration

struct Node
    x::Int # index in horizontal direction
    y::Int # index in vertical direction
    visited::Bool 
    is_wall::Bool # determines wether node can be part of a path 
end

# finds the neighbors of a list of all neighboring nodes
function neighbors(node::Node, grid::Array{Node, 2})
    nbrs = []
    # checkingy direction all desired directions (here diagonals disregarded)
    for (dx, dy) in ((-1, 0), (1, 0), (0, -1), (0, 1))
        nx, ny = node.x + dx, node.y + dy
        if nx > 0 && ny > 0 && nx <= size(grid, 1) && ny <= size(grid, 2)
            push!(nbrs, grid[nx, ny])
        end
    end
    return nbrs
end

struct Maze
    nodes::Matrix{Node} # Nodes will be stored in 2D array to determine neighbors 
    visual::Union{Nothing, MazeViz} # Visualization of the graph
    path::Union{Nothing, Vector{Node}} # path that solves the labyrinth
end

# Maze constructor
function Maze(height::Int, width::Int)
    nodes = [Node(x, y, false, false) for x in 1:height, y in 1:width]
    maze = Maze(nodes, nothing, nothing)
    generate_maze!(maze)
    maze.visual = visualize_maze(maze)
    solve(maze)
    return maze
end

function generate_maze!(maze::Maze)
    height, width = size(maze.nodes)
    stack = []

    # Start with a random node
    current = maze.nodes[rand(1:height), rand(1:width)]
    current.visited = true
    push!(stack, current)

    while !isempty(stack)
        current = stack[end]
        unvisited_neighbors = filter(n -> !n.visited, neighbors(current, maze.nodes))
        if !isempty(unvisited_neighbors)
            next_node = rand(unvisited_neighbors)
            next_node.visited = true
            push!(stack, next_node)
        else
            pop!(stack)
        end
    end
end

include("solver.jl")
include("visualize.jl")

using .Solver
using .Visualize

Base.show(io::IO, maze::Maze) = println(io, maze.visual.representation)

end
=#