module Visualize

using ..Core

export MazeViz, visualize_maze

struct MazeViz
    representation::String
end

function visualize_maze(maze::Maze)
    height, width = size(maze.nodes)
    representation = ""
    for i in 1:height
        for j in 1:width
            representation *= maze.nodes[i, j].visited ? " " : "█"
        end
        representation *= "\n"
    end
    return MazeViz(representation)
end

end # module
