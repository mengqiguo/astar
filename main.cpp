#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

struct Point {
    int x;
    int y;

    bool operator==(const Point& other) const {
        return x == other.x && y == other.y;
    }
};

struct Node {
    int x;
    int y;
    double f;

    bool operator>(const Node& other) const {
        return f > other.f;
    }
};

static double heuristic(const Point& a, const Point& b) {
    // Manhattan distance works well for 4-neighbor grid.
    return static_cast<double>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
}

static bool inBounds(int x, int y, int rows, int cols) {
    return x >= 0 && x < rows && y >= 0 && y < cols;
}

static std::vector<Point> reconstructPath(const std::vector<std::vector<Point>>& parent,
                                          Point start,
                                          Point goal) {
    std::vector<Point> path;
    Point cur = goal;

    while (!(cur == start)) {
        path.push_back(cur);
        Point p = parent[cur.x][cur.y];
        if (p.x == -1 && p.y == -1) {
            return {};
        }
        cur = p;
    }
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

static std::vector<Point> aStar(const std::vector<std::vector<int>>& grid, Point start, Point goal) {
    const int rows = static_cast<int>(grid.size());
    const int cols = static_cast<int>(grid[0].size());

    const double INF = std::numeric_limits<double>::infinity();
    std::vector<std::vector<double>> gScore(rows, std::vector<double>(cols, INF));
    std::vector<std::vector<double>> fScore(rows, std::vector<double>(cols, INF));
    std::vector<std::vector<bool>> closed(rows, std::vector<bool>(cols, false));
    std::vector<std::vector<Point>> parent(rows, std::vector<Point>(cols, {-1, -1}));

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;

    gScore[start.x][start.y] = 0.0;
    fScore[start.x][start.y] = heuristic(start, goal);
    openSet.push({start.x, start.y, fScore[start.x][start.y]});

    const int dx[4] = {-1, 1, 0, 0};
    const int dy[4] = {0, 0, -1, 1};

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        if (closed[current.x][current.y]) {
            continue;
        }
        closed[current.x][current.y] = true;

        if (current.x == goal.x && current.y == goal.y) {
            return reconstructPath(parent, start, goal);
        }

        for (int i = 0; i < 4; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (!inBounds(nx, ny, rows, cols)) {
                continue;
            }
            if (grid[nx][ny] == 1 || closed[nx][ny]) {
                continue;
            }

            double tentativeG = gScore[current.x][current.y] + 1.0;
            if (tentativeG < gScore[nx][ny]) {
                parent[nx][ny] = {current.x, current.y};
                gScore[nx][ny] = tentativeG;
                fScore[nx][ny] = tentativeG + heuristic({nx, ny}, goal);
                openSet.push({nx, ny, fScore[nx][ny]});
            }
        }
    }

    return {};
}

static void printPathPoints(const std::vector<Point>& path) {
    std::cout << "规划轨迹点 (x, y):\n";
    for (std::size_t i = 0; i < path.size(); ++i) {
        std::cout << i << ": (" << path[i].x << ", " << path[i].y << ")\n";
    }
}

static void printVisualization(const std::vector<std::vector<int>>& grid,
                               const std::vector<Point>& path,
                               Point start,
                               Point goal) {
    std::vector<std::vector<char>> canvas(grid.size(), std::vector<char>(grid[0].size(), ' '));

    for (std::size_t i = 0; i < grid.size(); ++i) {
        for (std::size_t j = 0; j < grid[0].size(); ++j) {
            canvas[i][j] = (grid[i][j] == 1) ? '#' : '.';
        }
    }

    for (const auto& p : path) {
        if (!(p == start) && !(p == goal)) {
            canvas[p.x][p.y] = '*';
        }
    }

    canvas[start.x][start.y] = 'S';
    canvas[goal.x][goal.y] = 'G';

    std::cout << "\n可视化轨迹:\n";
    std::cout << "图例: S=起点, G=终点, #=障碍, *=路径, .=可通行\n\n";

    for (const auto& row : canvas) {
        for (char c : row) {
            std::cout << c << ' ';
        }
        std::cout << '\n';
    }
}

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif

    std::vector<std::vector<int>> grid = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0, 1, 1, 1, 1, 0},
        {0, 0, 0, 1, 0, 0, 0, 0, 1, 0},
        {0, 1, 0, 1, 1, 1, 1, 0, 1, 0},
        {0, 1, 0, 0, 0, 0, 1, 0, 1, 0},
        {0, 1, 1, 1, 1, 0, 1, 0, 1, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {1, 1, 1, 0, 1, 1, 1, 1, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 1, 1, 1, 1, 1, 1, 0}
    };

    Point start{0, 0};
    Point goal{9, 9};

    std::vector<Point> path = aStar(grid, start, goal);

    if (path.empty()) {
        std::cout << "未找到可行路径。\n";
        //new branch for review
        //change
        std::cout << "未找到可行路径。\n";
        return 0;
    }

    printPathPoints(path);
    std::cout << "\n路径长度(步数): " << (path.size() - 1) << "\n";
    printVisualization(grid, path, start, goal);

    return 0;
}
