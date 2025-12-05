# turtlebot3_llm_semantic_navigation
大規模言語モデルを用いた自然言語ナビゲーションシステム

---

# 1. プロジェクト概要

本プロジェクトは，自然言語のみで移動ロボットを制御する新しいヒューマン・ロボット・インタラクション手法を提案する。
TurtleBot3 と ROS2 を基盤とし，Google Gemini などの大規模言語モデル（LLM）を用いて，
ユーザ指示をロボット行動計画（移動・旋回・停止など）へ自動変換するシステムを実装した。

Key idea
```
User → Natural Language → LLM → Semantic Plan → Robot executes
```
従来の「座標指定」や「低レベル制御コマンド」と異なり，
本研究では意味的な指示（例：窓の近くへ移動して）を基点とした操作を目指す。

---

# 2. 研究動機

従来のロボットナビゲーションは、数値ベース（x,y 座標）、低レベル命令（速度指令、角度指定）、使用者に専門知識が必要という問題があった。

しかし，人間は「前に進んで，右の壁の近くで止まって」のように意味レベルで移動指示を行う。

LLM の登場により，
ロボットが自然言語を理解し，自己判断で行動計画を生成する可能性が生まれた。

本研究では，その第一段階として：

### 自然言語 → 行動計画（JSON）→ ROS2 実行

への変換パイプラインを構築し，
Semantic Navigation への基礎的アーキテクチャを確立する。

---

# 3. システム構成

<img width="401" height="256" alt="名称未設定ファイル drawio" src="https://github.com/user-attachments/assets/ea119334-7991-40bf-a33d-371c4b16db92" />

今後の拡張として，本構造は LV2 / LV3 の意味的地図構築や視覚情報統合に対応可能である。

---

# 4. 動作原理

## 4-1. Natural Language → JSON Plan

ユーザ例：

```text
少し前に進んでから左に曲がって
```

```json
{
  "plan": [
    {"action": "FORWARD", "value": 0.5},
    {"action": "TURN_LEFT", "value": 90},
    {"action": "STOP", "value": 0}
  ]
}
```

## 4-2. JSON Plan → TurtleBot3 Control

ROS2 ノード llm_controller.py は—

各アクションの実行時間を算出

/cmd_vel に速度指令を発行

計画を逐次実行

ユーザはコマンドを入力するだけでよい。

---

# 5. 使い方

## 5-1. ビルド

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 5-2. 実行

```bash
ros2 run tb3_llm_controller llm_controller
```

## 3. 指示例

```text
少し前に進んで
前に進んで右に曲がって
左に90度回転してから止まって
壁の近くまで行って止まって
```

---

# 6. 実行例




ロボットがユーザの日本語指示を受け取り，
LLM が生成した JSON 行動計画に従って自律的に動作する様子を確認した。

---

# 7. 今後の研究展開

現在，本システムは Semantic Navigation の LV1 を達成した。
次の段階として以下を計画している：

LV2：視覚セマンティック認識
    カメラ + YOLO による物体検出
    「机の近くへ」「ドア側に移動」など，
    視覚情報を用いた意味的ナビゲーション

LV3：環境理解と抽象指示推論
    LLM に環境情報（障害物・可視物体など）を入力
    「安全な場所」「空いている場所」など，
    抽象的指示に対する意味推論と行動理由の生成

---
