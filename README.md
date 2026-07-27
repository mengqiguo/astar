# A* Pathfinding Demo (C++)

一个基于二维栅格地图的 A* 路径规划示例，使用 C++ 实现。

## 功能

- 4 邻域（上/下/左/右）搜索
- 启发函数：Manhattan Distance
- 支持障碍物地图
- 输出路径点序列
- 终端字符可视化路径

## 项目结构

- `main.cpp`：A* 核心实现与示例入口
- `.gitignore`：忽略可执行文件（`*.exe`）

## 编译与运行

### 使用 g++

```bash
g++ -std=c++17 -O2 -o astar_demo main.cpp
./astar_demo
```

### Windows (PowerShell)

```powershell
g++ -std=c++17 -O2 -o astar_demo.exe main.cpp
.\astar_demo.exe
```

## 说明

地图中符号含义：

- `S`：起点
- `G`：终点
- `#`：障碍
- `*`：路径
- `.`：可通行
