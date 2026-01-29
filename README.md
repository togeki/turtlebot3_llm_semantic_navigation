# TurtleBot3 データパイプライン & 可視化システム

ROS2（Humble）と PostgreSQL を用いて、
ロボットの実験データを収集・保存・分析・可視化する
エンドツーエンドのデータ基盤を構築した。

本プロジェクトは、
ロボット実験を「データエンジニアリングの視点」で再設計することを目的とする。

---

# 1. 概要（Overview）

本システムは以下の流れで構成されている。

ROS2 → PostgreSQL → KPI 計算 → Web ダッシュボード

- ROS2 上のトピック（/odom, /cmd_vel, ナビゲーション結果）を収集
- PostgreSQL に run 単位 + 時系列データとして保存
- 実行結果から自動的に KPI を算出
- Streamlit により可視化・比較・分析を可能にする

---

# 2. システム構成（Architecture）

<img width="1024" height="1536" alt="ChatGPT Image 2026年1月29日 20_53_11" src="https://github.com/user-attachments/assets/274401c4-6963-4e11-803a-49097c41e2d5" />

---

# 3.データベース設計（Database Schema）

本システムでは以下の 3 テーブルを使用する。

1. runs  
   実験 1 回分のメタ情報および KPI を保持

2. events  
   LLM 指示、ナビゲーション目標、結果などのイベント履歴

3. timeseries_messages  
   /odom, /cmd_vel などの時系列トピックデータ

---

# 4. KPI 定義（評価指標）

本研究では以下の指標を自動算出する。

| 指標名 | 説明 |
|------|------|
| duration_s | 実行時間 |
| path_length_m | 走行距離 |
| avg_speed_mps | 平均速度 |
| stop_ratio | 停止状態の割合 |

---

# 5. ダッシュボード機能

- 実験一覧管理
- 成功・失敗の可視化
- 軌跡（x-y）表示
- 制御信号（/cmd_vel）分析
- KPI ランキング表示（最短時間・最短距離・最安定走行）

<img width="1885" height="951" alt="image" src="https://github.com/user-attachments/assets/094475eb-16d6-4d5d-a1f4-e65aa7cefdb5" />

---

# 6. 6. 技術スタック

- ROS2 Humble
- Nav2
- PostgreSQL
- Python (psycopg, pandas)
- Streamlit

---
