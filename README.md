# TurtleBot3 実験データパイプライン & 可視化システム

ROS2（Humble）と PostgreSQL を用いて、
ロボットの実験データを収集・保存・分析・可視化する
エンドツーエンドのデータ基盤を構築した。

本プロジェクトは、
ロボット実験を「データエンジニアリングの視点」から再設計し、
再現性と定量評価を可能とする実験基盤を構築することを目的とする。

---

# 1. 概要（Overview）

本システムは以下の流れで構成されている。

ROS2 → PostgreSQL → KPI 計算 → Web ダッシュボード

- ROS2 上のトピック（/odom, /cmd_vel, ナビゲーション結果）を収集
- PostgreSQL に run 単位 + 時系列データとして保存
- 実行結果から自動的に KPI を算出
- Streamlit により可視化・比較・分析を可能にする

これにより、ロボットの動作を単なるログではなく、
「構造化された実験データ」として管理・比較・分析することが可能となる。

---

# 2. システム構成（Architecture）

<img width="1024" height="1536" alt="ChatGPT Image 2026年1月29日 20_53_11" src="https://github.com/user-attachments/assets/274401c4-6963-4e11-803a-49097c41e2d5" />

---

# 3.データベース設計（Database Schema）

本システムでは以下の 3 テーブルを中心としたデータベース設計を採用する。

1. runs  

   実験 1 回分の実行単位（run）を表し、開始時刻・終了時刻・実行結果・KPI を保持する。

3. events  

   ナビゲーション指令、目標地点、結果通知などのイベント履歴を時系列で記録する。

4. timeseries_messages

   /odom, /cmd_vel などの ROS トピックを時系列データとして保存する。

---

# 4. KPI 定義（評価指標）

これらの指標は、ナビゲーション性能を定量的に評価するために設計したものであり、
run 終了時に自動的に計算され、データベースに保存される。

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
<img width="1884" height="252" alt="image" src="https://github.com/user-attachments/assets/fdf65c82-995e-4537-900e-4ad08df8ecaf" />

---

# 6. 技術スタック

- ROS2 Humble
- Nav2
- PostgreSQL
- Python (psycopg, pandas)
- Streamlit

---

# 7. まとめ

本プロジェクトでは、TurtleBot3 を用いたロボット実験を対象に、

- ROS2 による実行制御
- PostgreSQL によるデータ永続化
- KPI による定量評価
- ダッシュボードによる可視化

を統合した実験データ基盤を構築した。

本構成により、ロボット実験を
「再現性のあるデータパイプライン」として扱うことが可能となり、
研究用途および実運用システムの双方に応用可能な基盤を実現した。

---
