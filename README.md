<h1 align="center">🧭 Informed Search Visualizer</h1>

<p align="center">
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white" />
  <img src="https://img.shields.io/badge/Pygame-90EE90?style=for-the-badge" />
</p>

<p align="center"><b>A Pygame-based grid visualizer for heuristic-guided pathfinding algorithms.</b></p>

---

## 📖 Project Explanation

This application renders an animated grid with a dedicated sidebar control panel for running informed search algorithms. Pathfinding is guided by heuristic cost functions — Manhattan distance and Euclidean distance — and the grid distinguishes between the start cell, goal cell, static and dynamically spawned walls, visited nodes, the active frontier queue, the moving agent, and the final computed path, each rendered in a distinct color.

## 🛠️ Tech Stack

<p>
  <img src="https://img.shields.io/badge/Language-Python-3776AB?style=flat-square&logo=python" />
  <img src="https://img.shields.io/badge/Graphics-Pygame-90EE90?style=flat-square" />
</p>

## 🧩 Things Used

- **Pygame** — window management, rendering, and the game loop
- **`heapq`** — priority queue implementing the frontier for cost-based search
- **Manhattan & Euclidean distance heuristics** — guide the search toward the goal more efficiently than blind search

## 📂 Project Structure

```
.
└── code_file.py    # Application window, grid rendering, heuristics, and search logic
```

## 🚀 Setup & Usage

```bash
pip install pygame
python code_file.py
```

## ⭐ Honest Review

**Strengths:** The sidebar UI with hover and active-state styling shows real attention to interface polish beyond just "make the algorithm work" — that level of finish is uncommon in coursework visualizers and makes the project noticeably more presentable in a portfolio or demo setting.

**Areas for improvement:** The main script's filename (`code_file.py`) is generic and worth renaming to something descriptive like `pathfinder.py` for clarity. As with the companion uninformed-search project, this would benefit from being combined into a single repository covering the full search-algorithms unit, giving a reviewer one clear, complete project to look at instead of two related-but-separate ones.

## 👤 Author

Ayaan Amir — Artificial Intelligence coursework project.

## 📄 License

MIT
