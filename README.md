# LLMを用いたTurtleBot3屋内自律巡回システム

## 1. プロジェクト概要

本プロジェクトでは、大規模言語モデル（LLM）とROS 2のナビゲーション機能を組み合わせ、自然言語による指示で屋内を自律移動するロボットシステムを構築した。

ユーザーが日本語で「ポイントCへ移動してください」「家の中の4か所を順番に巡回してください」などと指示すると、Google Geminiが行動計画をJSON形式で生成する。

生成された計画をROS 2ノードが解釈し、Nav2による経路計画・障害物回避・目的地への移動を実行する。

シミュレーションにはTurtleBot3 BurgerとGazebo House環境を使用した。

本システムは、事前に登録した巡回地点を利用する方式であり、画像認識による物体や部屋の自動識別は実装していない。

## 2. 開発環境

| 項目 | 使用技術 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| シミュレーター | Gazebo Sim 8 |
| ロボット | TurtleBot3 Burger |
| 言語モデル | Google Gemini 2.5 Flash |
| プログラミング言語 | Python 3 |
| 自律移動 | Navigation2（Nav2） |
| 自己位置推定 | AMCL |
| 地図 | Gazebo Houseから生成した占有格子地図 |
| 可視化 | RViz2 |

## 3. システム構成

自然言語の指示からロボットの移動まで、以下の処理を行う。

1. ユーザーが日本語の指示を入力する。
2. Geminiが指示を解釈し、JSON形式の行動計画を生成する。
3. `llm_controller` が各行動を順番に実行する。
4. `semantic_place_bridge` が巡回地点名を座標に変換する。
5. `llm_nav_bridge` が座標を `PoseStamped` に変換する。
6. `nav2_client` がNav2の `NavigateToPose` Actionに目標を送信する。
7. ナビゲーション結果をコントローラーに返し、成功した場合に次の地点へ進む。

主なROS 2トピック：

| トピック | 役割 |
|---|---|
| `/llm_place` | 目的地名の送信 |
| `/llm_nav_json` | 目的地座標の送信 |
| `/go_to_pose` | Nav2向け目標Poseの送信 |
| `/tb3_nav_result` | ナビゲーション結果の通知 |
| `/cmd_vel` | ロボットの速度制御 |

ナビゲーション中は、LLMコントローラーからの速度指令を停止し、Nav2に移動制御を任せる。

## 4. 巡回地点

Gazebo Houseの占有格子地図上に巡回地点を設定した。

| 地点 | X（m） | Y（m） |
|---|---:|---:|
| point_c | 6.330 | -4.261 |
| point_a | 1.305 | 3.407 |
| point_e | -2.365 | 4.588 |
| point_b | -6.410 | -2.427 |

巡回順序：

**point_c → point_a → point_e → point_b**

これらの地点は、RViz2の地図上で選択し、シミュレーション環境で移動を試験した位置である。物体認識により自動検出した地点ではない。

地点情報は `semantic_places.yaml` に保存する。

## 5. 自然言語による行動計画

### 単一地点への移動

入力：

`ポイントCへ移動してください`

生成される行動計画の例：

```json
[
  {"action": "GO_TO_PLACE", "target": "point_c"},
  {"action": "STOP", "value": 0}
]
```

### 複数地点の巡回

入力：

`家の中の4か所を順番に巡回してください`

生成される行動計画：

```json
[
  {"action": "GO_TO_PLACE", "target": "point_c"},
  {"action": "GO_TO_PLACE", "target": "point_a"},
  {"action": "GO_TO_PLACE", "target": "point_e"},
  {"action": "GO_TO_PLACE", "target": "point_b"},
  {"action": "STOP", "value": 0}
]
```

各地点への移動ではNav2が経路を計画する。コントローラーはナビゲーションの成功通知を受け取った後に次の行動へ進む。

## 6. セットアップと実行方法

前提条件：

- ROS 2 Jazzy
- TurtleBot3関連パッケージ
- Gazebo SimおよびROS-Gazebo Bridge
- Nav2、`turtlebot3_navigation2`
- Python環境と `google-genai`
- Gemini APIキー

### 6.1 ビルド

リポジトリのルートディレクトリで実行する。

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-select tb3_llm_controller --symlink-install
cd ..
```

### 6.2 Gazebo Houseを起動

```bash
bash scripts/sim.sh
```

### 6.3 Nav2とRViz2を起動

別のターミナルで実行する。

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  map:=/path/to/repository/maps/house.yaml \
  use_sim_time:=true
```

`/path/to/repository` は実際のリポジトリのパスに置き換える。

RViz2で **2D Pose Estimate** を使用し、ロボットの初期位置を設定する。

### 6.4 ナビゲーション関連ノードを起動

別のターミナルで実行する。

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 run tb3_llm_controller semantic_place_bridge &
ros2 run tb3_llm_controller llm_nav_bridge &
ros2 run tb3_llm_controller nav2_client &

wait
```

### 6.5 Geminiコントローラーを起動

ROS 2 JazzyのPythonパッケージと `google-genai` を利用できるPython環境が必要である。

Gemini APIキーを環境変数 `GEMINI_API_KEY` に設定する。APIキーをソースコードに直接記述しないこと。

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

python -m tb3_llm_controller.llm_controller
```

実行後、日本語の指示を入力する。

※ `python` は必要な依存パッケージがインストールされたPython環境の実行コマンドに置き換える。

## 7. 実験結果

Gazebo House環境で、以下の動作を確認した。

- 日本語の自然言語指示からGeminiがJSON形式の行動計画を生成した。
- Nav2を用いて指定した巡回地点への自律移動を確認した。
- ナビゲーション成功時に `SUCCEEDED` を受信し、コントローラーが次の処理に進む動作を確認した。
- 複数地点を順次訪問する巡回処理を実装し、シミュレーションで試験した。

一方、狭い通路付近で移動が停滞する場合があり、巡回全体を常に安定して完了できる状態には至っていない。

## 8. 課題と今後の改善

- **狭い通路での移動**：Nav2のCostmap、Inflation Layer、ロボットサイズなどのパラメータを調整する。
- **巡回の信頼性**：ナビゲーション失敗時の再試行や代替経路の処理を追加する。
- **地点の自動認識**：将来的にはカメラや物体検出モデルを利用し、環境中の物体と地点情報を対応付ける。
- **動作の安定性**：仮想環境における描画負荷やシミュレーション実行性能を改善する。

## 9. まとめ

本プロジェクトでは、LLMによる自然言語の行動計画生成と、Nav2による自律移動を連携させた屋内巡回システムを構築した。

従来の単純な前進・回転指示だけでなく、ユーザーが指定した地点への移動や、複数地点を順番に訪問するタスクを扱えるようにした。

今後は狭い通路でのナビゲーションの安定化と、視覚認識を利用した地点情報の自動取得を課題とする。
