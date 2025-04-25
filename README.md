# Snake AI Game

## 🎯 Project Goal

The goal of this project is to build an AI model for a snake that can **eat apples** and **avoid collisions** with another snake to **win the game**.

The first step toward achieving this is to implement a **search algorithm**, and we've chosen the **A\* (A-star) algorithm**, which is a foundational method for pathfinding in a given space.

---

## 🛠 Prerequisites

- Python
- Jupyter Notebook (or the Jupyter extension in VS Code)

---

## 📦 Install Dependencies

To install the required Python libraries, run:

```
pip install -r requirements.txt
```

---

## 🎮 Game Rules

- There are **two snakes**: **Red** and **Yellow**.
- Each snake has its own server:
  - Red snake: `rss.py`
  - Yellow snake: `yss.py`
- Each snake server receives the following information:
  - Its own head position
  - The opponent’s head position
  - The apple’s position

### Objectives

- Avoid colliding with the opponent's body.
- It's allowed to move into its own body.
- If a snake collides with the opponent, the opponent wins (similar to traditional snake rules).
- Both snakes start with a length of 5 and grow upon eating an apple.
- After eating an apple, a snake gains **temporary immunity**, allowing it to survive collisions with the opponent's body.

### Game Period

- The **game period** is the number of steps after which the game server queries the snake servers for their next move.
- A lower game period (e.g., 1) means the game is simpler. As the period increases, the game becomes more difficult since the AI must predict safe moves for multiple steps ahead.
- The snake servers must respond **within 1 second** to ensure smooth gameplay.

---

## 🚀 Running the Game Server

Open and run the game server notebook:

`snake_game.ipynb`

You can run this using Jupyter Notebook or the Jupyter extension in VS Code.

---

## 🐍 Running the Snake Servers

### Red Snake

To run the Red Snake server, use the following command:

```
python rss.py <game_period> <game_immunity>
```

Example:

```
python rss.py 1 1
```

- `<game_period>`: The number of steps after which the game server queries the snake server for its next move.
- `<game_immunity>`: The duration (in steps) for which the snake gains temporary immunity after eating an apple.

### Yellow Snake

To run the Yellow Snake server, use the following command:

```
python yss.py <game_period> <game_immunity>
```

Example:

```
python yss.py 1 1
```

- `<game_period>`: The number of steps after which the game server queries the snake server for its next move.
- `<game_immunity>`: The duration (in steps) for which the snake gains temporary immunity after eating an apple.

---