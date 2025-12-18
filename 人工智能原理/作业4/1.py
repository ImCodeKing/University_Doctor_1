# -*- coding: utf-8 -*-
"""
批量生成八数码问题（8-puzzle）和八皇后问题（8-queens），使用：
 - 爬山法（最陡上升 / Steepest-ascent hill climbing）
 - 爬山法（首选改进 / First-choice hill climbing）
 - 模拟退火（Simulated Annealing）

对每个问题实例计算：
 - 是否成功（是否找到可接受解 / 满足目标）
 - 算法返回解的代价（8-puzzle: 步数；8-queens: 冲突对数）
 - 搜索耗散（节点扩展计数或评估次数）
 - 与最优代价（对于8-puzzle用A*求最短步数；对于8-queens最优代价为0）比较

生成对比图：
 - 算法解代价 vs 最优代价（散点图 + y=x 参考线）
 - 每个算法的成功率条形图
 - 平均搜索耗散条形图

说明：
 - 代码全部为 Python 标准库 + numpy + matplotlib + heapq
 - 运行参数在 main() 中可配置：样本数、最大迭代、重启次数等
 - 请在有足够计算资源的机器上运行（A* 可能耗时，样本数太大请调小）
"""

import random
import math
import time
from collections import deque, defaultdict
import heapq
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager


def _configure_plot_style():
    """Configure matplotlib style and fonts so Chinese text renders correctly."""
    try:
        plt.style.use("seaborn-v0_8")
    except OSError:
        plt.style.use("ggplot")
    preferred_fonts = [
        "Microsoft YaHei",
        "SimHei",
        "Noto Sans CJK SC",
        "WenQuanYi Micro Hei",
        "Arial Unicode MS",
    ]
    available_fonts = {f.name for f in font_manager.fontManager.ttflist}
    for font_name in preferred_fonts:
        if font_name in available_fonts:
            plt.rcParams["font.family"] = font_name
            break
    else:
        plt.rcParams["font.family"] = "DejaVu Sans"
    plt.rcParams["font.sans-serif"] = preferred_fonts + ["DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False


def _apply_common_ax_style(ax):
    ax.set_facecolor("#fbfbfb")
    ax.grid(axis="y", linestyle="--", linewidth=0.8, alpha=0.35)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.set_axisbelow(True)


def _annotate_bars(ax, bars, labels, offset_ratio=0.015):
    if not bars:
        return
    ylim = ax.get_ylim()
    offset = (ylim[1] - ylim[0]) * offset_ratio
    for bar, label in zip(bars, labels):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + offset,
            label,
            ha="center",
            va="bottom",
            fontsize=10,
        )


def _get_color_map(labels, cmap_name="Set2"):
    if not labels:
        return {}
    cmap = plt.get_cmap(cmap_name)
    positions = np.linspace(0.15, 0.85, len(labels))
    colors = cmap(positions)
    return {label: colors[idx] for idx, label in enumerate(labels)}


def _save_figure(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)


_configure_plot_style()

# --------------------------
# 公共工具与评估函数
# --------------------------

def manhattan_distance(state):
    """8-puzzle 的曼哈顿距离（state 是长度 9 的序列或元组）"""
    dist = 0
    for idx, val in enumerate(state):
        if val == 0:
            continue
        target_row, target_col = divmod(val - 1, 3)
        row, col = divmod(idx, 3)
        dist += abs(row - target_row) + abs(col - target_col)
    return dist

def queens_conflicts(positions):
    """
    8-queens 冲突数计算。
    positions: 长度为 N 的列表，positions[col] = row（0-based）
    返回冲突对数（每对算一次）
    """
    n = len(positions)
    conflicts = 0
    for i in range(n):
        for j in range(i+1, n):
            if positions[i] == positions[j] or abs(positions[i]-positions[j]) == abs(i-j):
                conflicts += 1
    return conflicts

# --------------------------
# 八数码：生成可解拼图 / A* 最优解
# --------------------------

GOAL_STATE = tuple(range(1, 9)) + (0,)

def is_solvable(state):
    """判断8-puzzle是否可解（逆序数法则）"""
    arr = [x for x in state if x != 0]
    inv = 0
    for i in range(len(arr)):
        for j in range(i+1, len(arr)):
            if arr[i] > arr[j]:
                inv += 1
    # 对 3x3，逆序数为偶数则可解（目标为空格在最后）
    return inv % 2 == 0

def random_solvable_puzzle(shuffle_moves=30):
    """
    通过对目标状态执行若干随机合法移动生成可解拼图（避免复杂的逆序判断）
    shuffle_moves: 随机移动次数
    """
    state = list(GOAL_STATE)
    zero_pos = 8
    for _ in range(shuffle_moves):
        r, c = divmod(zero_pos, 3)
        neighbors = []
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r+dr, c+dc
            if 0 <= nr < 3 and 0 <= nc < 3:
                neighbors.append(nr*3 + nc)
        new_pos = random.choice(neighbors)
        state[zero_pos], state[new_pos] = state[new_pos], state[zero_pos]
        zero_pos = new_pos
    return tuple(state)

def generate_solvable_puzzles(n, shuffle_moves=30, seed=None):
    if seed is not None:
        random.seed(seed)
    puzzles = [random_solvable_puzzle(shuffle_moves) for _ in range(n)]
    return puzzles

def a_star_shortest_moves(start_state, max_expansions=200000):
    """
    使用 A*（启发函数：曼哈顿距离）求最短步数。
    返回 (found, moves, expansions)：
      - found: 是否找到解
      - moves: 最短步数（若未找到则为 None）
      - expansions: 节点扩展数（弹出/处理的节点数）
    注意：为了速度可设置 max_expansions 上限。
    """
    start = tuple(start_state)
    if start == GOAL_STATE:
        return True, 0, 0
    open_heap = []
    g = {start: 0}
    f = manhattan_distance(start)
    heapq.heappush(open_heap, (f, 0, start))
    expansions = 0
    seen = set()
    while open_heap:
        cur_f, _, cur = heapq.heappop(open_heap)
        if cur in seen:
            continue
        seen.add(cur)
        expansions += 1
        if expansions > max_expansions:
            return False, None, expansions
        if cur == GOAL_STATE:
            return True, g[cur], expansions
        # expand neighbors
        zero = cur.index(0)
        r, c = divmod(zero, 3)
        for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
            nr, nc = r+dr, c+dc
            if 0 <= nr < 3 and 0 <= nc < 3:
                new_pos = nr*3 + nc
                lst = list(cur)
                lst[zero], lst[new_pos] = lst[new_pos], lst[zero]
                nxt = tuple(lst)
                tentative_g = g[cur] + 1
                if nxt not in g or tentative_g < g[nxt]:
                    g[nxt] = tentative_g
                    h = manhattan_distance(nxt)
                    heapq.heappush(open_heap, (tentative_g + h, random.random()*1e-6, nxt))
    return False, None, expansions

# --------------------------
# 八皇后：生成随机初始配置
# --------------------------

def generate_random_queen_states(n, size=8, seed=None):
    if seed is not None:
        random.seed(seed)
    states = []
    for _ in range(n):
        # 每列随机放一个皇后（允许冲突）
        pos = [random.randrange(size) for __ in range(size)]
        states.append(pos)
    return states

# --------------------------
# 邻居生成函数（通用）
# --------------------------

def puzzle_neighbors(state):
    """返回 (neighbor_state, move_cost=1) 列表"""
    zero = state.index(0)
    r, c = divmod(zero, 3)
    neighs = []
    for dr, dc in ((1,0),(-1,0),(0,1),(0,-1)):
        nr, nc = r+dr, c+dc
        if 0 <= nr < 3 and 0 <= nc < 3:
            new_pos = nr*3 + nc
            lst = list(state)
            lst[zero], lst[new_pos] = lst[new_pos], lst[zero]
            neighs.append(tuple(lst))
    return neighs

def queens_neighbors(state):
    """
    对 8-queens 的邻居：在同一列内移动皇后到其它行（单步变动）
    返回所有邻居状态
    """
    n = len(state)
    neighs = []
    for col in range(n):
        for row in range(n):
            if row != state[col]:
                new = list(state)
                new[col] = row
                neighs.append(new)
    return neighs

# --------------------------
# 算法实现：爬山法 & 模拟退火（两问题通用接口）
# --------------------------

def hill_climb_steepest(start, neighbors_fn, eval_fn, max_iters=1000, allow_equal=False):
    """
    最陡上升爬山法
    - eval_fn 返回越小越好（需要最小化）
    - neighbors_fn(state) 返回邻居集合
    - allow_equal: 是否接受评价相等的邻居（默认否）
    返回 dict 包含: success(bool), state, value, steps_taken, eval_count
    """
    cur = start
    cur_val = eval_fn(cur)
    eval_count = 1
    steps = 0
    for it in range(max_iters):
        neighs = neighbors_fn(cur)
        best = None
        best_val = cur_val
        # 评估所有邻居，选择最小的
        for nst in neighs:
            v = eval_fn(nst)
            eval_count += 1
            if v < best_val or (allow_equal and v == best_val and best is None):
                best_val = v
                best = nst
        if best is None or best_val >= cur_val:
            break  # 局部最优，停止
        cur = best
        cur_val = best_val
        steps += 1
    return {"success": True, "state": cur, "value": cur_val, "steps_taken": steps, "eval_count": eval_count}

def hill_climb_first_choice(start, neighbors_fn, eval_fn, max_iters=1000, shuffle=True, sample_limit=None):
    """
    首选改进（First-choice hill climbing）
    - 随机检查邻居，发现比当前更优的立即接受
    - sample_limit: 每轮最多检查的邻居数量（None 表示全部随机顺序检查）
    返回与 steepest 相同格式的 dict
    """
    cur = start
    cur_val = eval_fn(cur)
    eval_count = 1
    steps = 0
    for it in range(max_iters):
        neighs = neighbors_fn(cur)
        if shuffle:
            random.shuffle(neighs)
        checked = 0
        improved = False
        for nst in neighs:
            v = eval_fn(nst)
            eval_count += 1
            checked += 1
            if v < cur_val:
                cur = nst
                cur_val = v
                steps += 1
                improved = True
                break
            if sample_limit is not None and checked >= sample_limit:
                break
        if not improved:
            break
    return {"success": True, "state": cur, "value": cur_val, "steps_taken": steps, "eval_count": eval_count}

def simulated_annealing(start, neighbors_fn, eval_fn, max_iters=5000, initial_temp=100.0, cooling_rate=0.995, min_temp=1e-3):
    """
    模拟退火（最小化问题）
    - 温度从 initial_temp 开始，每次乘以 cooling_rate
    - 接受概率：exp(-(v_new - v_cur)/T)（当 v_new > v_cur 时）
    返回 dict: success（是否达到目标判定外部给出）, state, value, steps, eval_count
    """
    cur = start
    cur_val = eval_fn(cur)
    eval_count = 1
    T = initial_temp
    steps = 0
    for it in range(max_iters):
        neighs = neighbors_fn(cur)
        nxt = random.choice(neighs)
        nxt_val = eval_fn(nxt)
        eval_count += 1
        delta = nxt_val - cur_val
        if delta < 0 or (T > 0 and random.random() < math.exp(-delta / T)):
            cur = nxt
            cur_val = nxt_val
            steps += 1
        T *= cooling_rate
        if T < min_temp:
            break
    return {"success": True, "state": cur, "value": cur_val, "steps_taken": steps, "eval_count": eval_count}

# --------------------------
# 评价 & 批量运行函数
# --------------------------

def evaluate_algorithms_on_puzzles(puzzles,
                                   algorithms,
                                   a_star_limit=200000,
                                   puzzle_max_iters=200,
                                   puzzle_restarts=5):
    """
    修正后的八数码批量评估：
      - 成功判定：到达 GOAL_STATE 或最终曼哈顿距离为 0
      - 记录字段：reached_goal, final_manhattan, eval_count, time
    """
    results = {name: [] for name in algorithms}
    optimums = []

    def _better_candidate(candidate, incumbent):
        if incumbent is None:
            return True
        if candidate["reached_goal"] != incumbent["reached_goal"]:
            return candidate["reached_goal"]
        cand_val = candidate["final_manhattan"]
        inc_val = incumbent["final_manhattan"]
        if cand_val is None:
            return False
        if inc_val is None:
            return True
        return cand_val < inc_val

    for p in puzzles:
        found, opt_moves, expansions = a_star_shortest_moves(p, max_expansions=a_star_limit)
        optimums.append({"solvable": found, "opt_moves": opt_moves, "a_star_expansions": expansions})
        if not found:
            for name in algorithms:
                results[name].append({"reached_goal": False, "final_manhattan": None, "eval_count": 0, "time": 0.0})
            continue

        for name, alg_fn in algorithms.items():
            best_record = None
            restarts = puzzle_restarts if ("hill" in name.lower() or "simulated" in name.lower()) else 1
            restarts = max(1, restarts)
            for _ in range(restarts):
                t0 = time.time()
                rec = alg_fn(p)
                rec_time = time.time() - t0

                final_state = rec.get("state")
                final_val = rec.get("value")
                reached_goal = False
                final_manhattan = None

                if final_state is not None:
                    tuple_state = tuple(final_state)
                    final_manhattan = manhattan_distance(tuple_state)
                    reached_goal = tuple_state == GOAL_STATE
                elif final_val is not None:
                    final_manhattan = final_val
                    reached_goal = final_manhattan == 0

                record = {
                    "reached_goal": bool(reached_goal),
                    "final_manhattan": final_manhattan,
                    "eval_count": rec.get("eval_count", 0),
                    "time": rec_time,
                }
                if _better_candidate(record, best_record):
                    best_record = record

            if best_record is None:
                best_record = {"reached_goal": False, "final_manhattan": None, "eval_count": 0, "time": 0.0}
            results[name].append(best_record)

    return {"optimums": optimums, "results": results}

def evaluate_algorithms_on_queens(states, size, algorithms, queen_max_iters=1000):
    """
    对一组八皇后实例批量评估算法。
    size: 皇后问题尺寸（通常为8）
    algorithms: dict 名称 -> 函数(start_state)
    """
    results = {name: [] for name in algorithms}
    optimums = []
    for s in states:
        opt_cost = 0  # 8-queens 的最优代价是 0（无冲突）
        optimums.append({"opt_cost": opt_cost})
        for name, alg_fn in algorithms.items():
            t0 = time.time()
            rec = alg_fn(s)
            rec_time = time.time() - t0
            results[name].append({"success": rec["value"] == 0, "cost": rec["value"], "eval_count": rec["eval_count"], "time": rec_time})
    return {"optimums": optimums, "results": results}

# --------------------------
# 为两个问题创建算法适配器（把通用算法包装成问题专用）
# --------------------------

# 8-puzzle 评价函数（目标是0）
def puzzle_eval_fn(state):
    return manhattan_distance(state)

def make_puzzle_algorithms(max_iters=200, sample_limit=None, sa_params=None):
    if sa_params is None:
        sa_params = {"max_iters": 2000, "initial_temp": 50.0, "cooling_rate": 0.995}
    algs = {}
    algs["puzzle_hill_steepest"] = lambda s: hill_climb_steepest(s, puzzle_neighbors, puzzle_eval_fn, max_iters=max_iters)
    algs["puzzle_hill_first_choice"] = lambda s: hill_climb_first_choice(s, puzzle_neighbors, puzzle_eval_fn, max_iters=max_iters, sample_limit=sample_limit)
    algs["puzzle_simulated_annealing"] = lambda s: simulated_annealing(s, puzzle_neighbors, puzzle_eval_fn, **sa_params)
    return algs

# 8-queens 评价函数（冲突对数）
def queen_eval_fn(state):
    return queens_conflicts(state)

def make_queen_algorithms(size=8, max_iters=1000, sa_params=None):
    if sa_params is None:
        sa_params = {"max_iters": 5000, "initial_temp": 30.0, "cooling_rate": 0.995}
    algs = {}
    algs["queen_hill_steepest"] = lambda s: hill_climb_steepest(s, queens_neighbors, queen_eval_fn, max_iters=max_iters)
    algs["queen_hill_first_choice"] = lambda s: hill_climb_first_choice(s, queens_neighbors, queen_eval_fn, max_iters=max_iters, sample_limit=None)
    algs["queen_simulated_annealing"] = lambda s: simulated_annealing(s, queens_neighbors, queen_eval_fn, **sa_params)
    return algs

# 图例显示名称映射
PUZZLE_ALGO_LABELS = {
    "puzzle_hill_steepest": "最陡上升爬山",
    "puzzle_hill_first_choice": "首选改进爬山",
    "puzzle_simulated_annealing": "模拟退火",
}

QUEEN_ALGO_LABELS = {
    "queen_hill_steepest": "最陡上升爬山",
    "queen_hill_first_choice": "首选改进爬山",
    "queen_simulated_annealing": "模拟退火",
}

# --------------------------
# 可视化与对比
# --------------------------

def plot_puzzle_results(puzzles, eval_data, save_prefix="puzzle"):
    """
    绘制：
     - 终止曼哈顿距离 vs 最优步数（散点图）
     - 各算法成功率条形图
     - 各算法平均搜索耗散条形图
    """
    optimums = eval_data["optimums"]
    results = eval_data["results"]
    algo_names = list(results.keys())
    display_names = [PUZZLE_ALGO_LABELS.get(name, name) for name in algo_names]
    color_map = _get_color_map(algo_names)
    n = len(puzzles)

    # 收集最优代价
    opt_moves = [o["opt_moves"] if o["solvable"] else None for o in optimums]

    # 散点图：每个算法的终止曼哈顿距离 vs 最优步数（仅对可解样本）
    fig, ax = plt.subplots(figsize=(8, 6))
    opt_pool = [v for v in opt_moves if v is not None]
    manhattan_pool = []
    for name in algo_names:
        xs, ys = [], []
        for i in range(n):
            if opt_moves[i] is None:
                continue
            rec = results[name][i]
            final_mh = rec.get("final_manhattan")
            if final_mh is None:
                continue
            xs.append(opt_moves[i])
            ys.append(final_mh)
        if not xs:
            continue
        ax.scatter(
            xs,
            ys,
            label=PUZZLE_ALGO_LABELS.get(name, name),
            alpha=0.85,
            s=55,
            color=color_map.get(name),
            edgecolors="#ffffff",
            linewidth=0.8,
        )
        manhattan_pool.extend(ys)
    axis_x = max(opt_pool) if opt_pool else 1
    axis_y = max(manhattan_pool) if manhattan_pool else 0
    axis_x = max(1, axis_x) * 1.05
    axis_y = max(0.5, axis_y) * 1.05 if manhattan_pool else 1
    ax.axhline(
        0,
        linestyle="--",
        color="#555555",
        linewidth=1.2,
        label="曼哈顿=0（达到目标）",
    )
    ax.set_xlim(0, axis_x)
    ax.set_ylim(-0.5, axis_y)
    ax.set_xlabel("最优步数（A*）")
    ax.set_ylabel("算法终止曼哈顿距离")
    ax.set_title("八数码：终止曼哈顿 vs 最优步数")
    _apply_common_ax_style(ax)
    ax.legend(frameon=False)
    _save_figure(fig, f"{save_prefix}_cost_vs_opt.png")

    # 成功率与平均搜索耗散
    succ_rates = []
    avg_evals = []
    avg_times = []
    for name in algo_names:
        succ = 0
        total_eval = 0
        total_time = 0
        count = 0
        for i in range(n):
            if opt_moves[i] is None:
                continue
            rec = results[name][i]
            if rec["reached_goal"]:
                succ += 1
            total_eval += rec["eval_count"]
            total_time += rec["time"]
            count += 1
        succ_rates.append(succ / count if count else 0)
        avg_evals.append(total_eval / count if count else 0)
        avg_times.append(total_time / count if count else 0)

    bar_colors = [color_map.get(name) for name in algo_names]
    x_pos = np.arange(len(algo_names))

    # 成功率条形图
    fig, ax = plt.subplots(figsize=(8, 4))
    bars = ax.bar(x_pos, succ_rates, color=bar_colors)
    ax.set_ylabel("成功率（成功求解）")
    ax.set_title("八数码：算法成功率")
    ax.set_ylim(0, 1.05)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(display_names)
    _apply_common_ax_style(ax)
    _annotate_bars(ax, list(bars), [f"{rate:.0%}" for rate in succ_rates])
    _save_figure(fig, f"{save_prefix}_success_rate.png")

    # 平均评估次数条形图
    fig, ax = plt.subplots(figsize=(8, 4))
    bars = ax.bar(x_pos, avg_evals, color=bar_colors)
    ax.set_ylabel("平均评估次数")
    ax.set_title("八数码：平均搜索耗散（评估次数）")
    max_eval = max(avg_evals) if any(avg_evals) else 1
    ax.set_ylim(0, max(1, max_eval) * 1.25)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(display_names)
    _apply_common_ax_style(ax)
    _annotate_bars(ax, list(bars), [f"{val:.0f}" for val in avg_evals])
    _save_figure(fig, f"{save_prefix}_avg_evals.png")

    print(f"[保存] 八数码图像: {save_prefix}_cost_vs_opt.png, {save_prefix}_success_rate.png, {save_prefix}_avg_evals.png")

def plot_queen_results(states, eval_data, save_prefix="queen"):
    optimums = eval_data["optimums"]
    results = eval_data["results"]
    algo_names = list(results.keys())
    display_names = [QUEEN_ALGO_LABELS.get(name, name) for name in algo_names]
    color_map = _get_color_map(algo_names)
    n = len(states)

    # 散点图：算法代价 vs 最优代价（opt=0）
    fig, ax = plt.subplots(figsize=(8, 6))
    max_cost = 0
    for name in algo_names:
        costs = []
        for i in range(n):
            rec = results[name][i]
            if rec["cost"] is None:
                continue
            costs.append(rec["cost"])
        if not costs:
            continue
        xs = [random.uniform(-0.06, 0.06) for _ in costs]
        ax.scatter(
            xs,
            costs,
            label=QUEEN_ALGO_LABELS.get(name, name),
            alpha=0.85,
            s=55,
            color=color_map.get(name),
            edgecolors="#ffffff",
            linewidth=0.8,
        )
        max_cost = max(max_cost, max(costs))
    ax.set_xlim(-0.25, 0.25)
    ax.set_xticks([0])
    ax.set_xticklabels(["最优代价 0"])
    ax.set_ylabel("算法返回代价（冲突对数）")
    ax.set_title("八皇后：算法代价分布 (最优=0)")
    ax.axhline(0, linestyle="--", color="#555555", linewidth=1.2)
    ax.set_ylim(-0.5, max(1, max_cost) * 1.1)
    _apply_common_ax_style(ax)
    ax.legend(frameon=False)
    _save_figure(fig, f"{save_prefix}_cost_dist.png")

    # 成功率与平均搜索耗散
    succ_rates = []
    avg_evals = []
    avg_times = []
    for name in algo_names:
        succ = 0
        total_eval = 0
        total_time = 0
        count = 0
        for i in range(n):
            rec = results[name][i]
            if rec["success"]:
                succ += 1
            total_eval += rec["eval_count"]
            total_time += rec["time"]
            count += 1
        succ_rates.append(succ / count if count else 0)
        avg_evals.append(total_eval / count if count else 0)
        avg_times.append(total_time / count if count else 0)

    bar_colors = [color_map.get(name) for name in algo_names]
    x_pos = np.arange(len(algo_names))

    fig, ax = plt.subplots(figsize=(8, 4))
    bars = ax.bar(x_pos, succ_rates, color=bar_colors)
    ax.set_ylabel("成功率（找到无冲突解）")
    ax.set_title("八皇后：算法成功率")
    ax.set_ylim(0, 1.05)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(display_names)
    _apply_common_ax_style(ax)
    _annotate_bars(ax, list(bars), [f"{rate:.0%}" for rate in succ_rates])
    _save_figure(fig, f"{save_prefix}_success_rate.png")

    fig, ax = plt.subplots(figsize=(8, 4))
    bars = ax.bar(x_pos, avg_evals, color=bar_colors)
    ax.set_ylabel("平均评估次数")
    ax.set_title("八皇后：平均搜索耗散（评估次数）")
    max_eval = max(avg_evals) if any(avg_evals) else 1
    ax.set_ylim(0, max(1, max_eval) * 1.25)
    ax.set_xticks(x_pos)
    ax.set_xticklabels(display_names)
    _apply_common_ax_style(ax)
    _annotate_bars(ax, list(bars), [f"{val:.0f}" for val in avg_evals])
    _save_figure(fig, f"{save_prefix}_avg_evals.png")

    print(f"[保存] 八皇后图像: {save_prefix}_cost_dist.png, {save_prefix}_success_rate.png, {save_prefix}_avg_evals.png")

# --------------------------
# main 调度：配置参数并运行
# --------------------------

def main():
    random.seed(556)
    np.random.seed(556)

    # 配置：样本数与算法参数（可按需调整）
    puzzle_samples = 1000            # 八数码实例数量（A* 对每个样本较耗时，必要时降低）
    queen_samples = 1000            # 八皇后实例数量
    puzzle_shuffle_moves = 30      # 生成拼图的随机移动步数
    a_star_limit = 200000

    # 算法参数
    puzzle_algorithms = make_puzzle_algorithms(max_iters=500, sample_limit=None,
                                              sa_params={"max_iters":2000, "initial_temp":50.0, "cooling_rate":0.995, "min_temp":1e-3})
    queen_algorithms = make_queen_algorithms(size=8, max_iters=1000,
                                             sa_params={"max_iters":5000, "initial_temp":20.0, "cooling_rate":0.995, "min_temp":1e-3})

    print("生成八数码样本...")
    puzzles = generate_solvable_puzzles(puzzle_samples, shuffle_moves=puzzle_shuffle_moves, seed=123)
    print("开始评估八数码算法（含 A* 最优解搜索）...")
    puzzle_eval = evaluate_algorithms_on_puzzles(puzzles, puzzle_algorithms, a_star_limit=a_star_limit, puzzle_restarts=3)
    print("绘制八数码结果图...")
    plot_puzzle_results(puzzles, puzzle_eval, save_prefix="puzzle_results")

    print("生成八皇后样本...")
    queens = generate_random_queen_states(queen_samples, size=8, seed=456)
    print("评估八皇后算法...")
    queen_eval = evaluate_algorithms_on_queens(queens, 8, queen_algorithms)
    print("绘制八皇后结果图...")
    plot_queen_results(queens, queen_eval, save_prefix="queen_results")

    # 额外打印汇总统计
    print("\n--- 汇总统计（示例） ---")
    for name, recs in puzzle_eval["results"].items():
        final_vals = []
        solved_count = 0
        considered = 0
        for rec, opt in zip(recs, puzzle_eval["optimums"]):
            if not opt["solvable"]:
                continue
            considered += 1
            if rec["reached_goal"]:
                solved_count += 1
            if rec["final_manhattan"] is not None:
                final_vals.append(rec["final_manhattan"])
        avg_manhattan = sum(final_vals)/len(final_vals) if final_vals else None
        avg_eval = sum(r["eval_count"] for r in recs)/len(recs)
        success_rate = solved_count / considered if considered else 0.0
        avg_m_text = f"{avg_manhattan:.2f}" if avg_manhattan is not None else "N/A"
        print(f"{name}: 成功率={success_rate:.2%} 平均终止曼哈顿={avg_m_text} 平均评估次数={avg_eval:.1f}")

    for name, recs in queen_eval["results"].items():
        succ = sum(1 for r in recs if r["success"])
        print(f"{name} 八皇后成功率 {succ}/{len(recs)} = {succ/len(recs):.2f}")

if __name__ == "__main__":
    main()
