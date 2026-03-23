/**
 * g++ -std=c++17 -O2 bfs_pathfinder.cpp -o bfs_pathfinder
 * ./bfs_pathfinder.exe
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <climits>
#include <cctype>

struct Point {
    int r, c;
    Point(int r = 0, int c = 0) : r(r), c(c) {}
    bool operator==(const Point& other) const {
        return r == other.r && c == other.c;
    }
};

int main() {
    // 1. 读取地图文件
    std::ifstream fin("map.txt");
    if (!fin) {
        std::cerr << "Error: Cannot open map.txt\n";
        return 1;
    }

    std::vector<std::string> grid;
    std::string line;
    while (std::getline(fin, line)) {
        if (!line.empty()) {
            grid.push_back(line);
        }
    }
    fin.close();

    if (grid.empty()) {
        std::cerr << "Error: map.txt is empty\n";
        return 1;
    }

    int H = grid.size();
    int W = grid[0].size();

    // 2. 找起点 S 和终点 E
    Point start(-1, -1), goal(-1, -1);
    for (int i = 0; i < H; ++i) {
        for (int j = 0; j < W; ++j) {
            if (grid[i][j] == 'S') {
                start = Point(i, j);
            } else if (grid[i][j] == 'E') {
                goal = Point(i, j);
            }
        }
    }

    if (start.r == -1 || goal.r == -1) {
        std::cerr << "Error: Missing S or E in map\n";
        return 1;
    }

    // 3. BFS 初始化
    std::vector<std::vector<int>> dist(H, std::vector<int>(W, -1)); // -1 表示未访问
    std::vector<std::vector<Point>> parent(H, std::vector<Point>(W, Point(-1, -1)));
    std::queue<Point> q;

    // 方向：上、右、下、左
    const int dr[4] = {-1, 0, 1, 0};
    const int dc[4] = {0, 1, 0, -1};

    dist[start.r][start.c] = 0;
    q.push(start);

    bool found = false;
    while (!q.empty()) {
        Point cur = q.front(); q.pop();
        if (cur == goal) {
            found = true;
            break;
        }

        for (int d = 0; d < 4; ++d) {
            int nr = cur.r + dr[d];
            int nc = cur.c + dc[d];
            if (nr >= 0 && nr < H && nc >= 0 && nc < W) {
                char ch = grid[nr][nc];
                // 可通行条件：是 '.'、'E'（终点），或者原本是 'S'（但已离开）
                if ((ch == '.' || ch == 'E' || ch == 'S' || ch == '?') && dist[nr][nc] == -1) {
                    // 注意：不能通过 '+' 或其他符号
                    if (ch != '+') {
                        dist[nr][nc] = dist[cur.r][cur.c] + 1;
                        parent[nr][nc] = cur;
                        q.push(Point(nr, nc));
                    }
                }
            }
        }
    }

    // 4. 输出结果
    if (found) {
        std::cout << "[BFS] Path found! Length = " << dist[goal.r][goal.c] << "\n";

        // 回溯路径并标记
        std::vector<std::string> result = grid;
        Point p = goal;
        while (!(p == start)) {
            if (result[p.r][p.c] != 'S' && result[p.r][p.c] != 'E') {
                result[p.r][p.c] = '*'; // 路径用 * 表示
            }
            p = parent[p.r][p.c];
        }

        // 打印带路径的地图
        for (const auto& row : result) {
            std::cout << row << "\n";
        }
    } else {
        std::cout << "[BFS] NO PATH FOUND!\n";
        // 也可以打印原地图
        for (const auto& row : grid) {
            std::cout << row << "\n";
        }
    }

    return 0;
}