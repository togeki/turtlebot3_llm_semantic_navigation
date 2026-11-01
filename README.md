# 自然言語による TurtleBot3 制御

日本語指示 → Gemini → JSON 行動計画 → ROS 2 → Gazebo Sim の TurtleBot3 Burger。
既存の `gemini_planner` と時間制御ノードを使い、Physical AI 課題用に前進・旋回・停止の再現手順を整理しました。

## 確認した環境と結果

2026-11-01、UTM 上の Ubuntu 24.04.3 / ROS 2 Jazzy / Gazebo Sim 8.10.0 / Python 3.12 で確認。

- ROS パッケージのビルド成功、計画検証テスト 13 件成功。
- 整理した launch から Gazebo GUI・Burger・ROS bridge の起動成功。
- `/cmd_vel` は `geometry_msgs/msg/TwistStamped`。旧 Twist 発行コードを対応させた。
- 同じ制御ノードによる固定計画：前進 0.2 m → 右転 90° → 停止。`/odom` 測定は約 0.195 m、−88.8°、終了時速度はほぼ 0。
- 実際の Gemini 2.5 Flash に日語指示を送り、前進 0.2 m・右転 90°・停止の JSON を取得して実行成功。AI 計画の実測は前進 **0.19524 m**、右転 **88.77°**、終了時の線速度・角速度は **0**。初回 API 呼び出しは ServerError、再試行後は成功した。
- 提出動画はまだ未収録。上記は実行ログと里程計による確認結果。

## 構成

```text
scripts/setup.sh              隔離 venv と ROS パッケージのビルド
scripts/sim.sh                Jazzy シミュレーションの起動
scripts/controller.sh         Key を表示せず対話コントローラを起動
scripts/verify_motion.py       実機構成の /odom による固定計画確認（AI なし）
ros2_ws/src/tb3_llm_controller/
  launch/demo_sim.launch.py    インストール済み ROBOTIS の world / model を使用
  tb3_llm_controller/
    gemini_planner.py          API 呼び出し・JSON 検証
    llm_controller.py          時間制御による逐次実行
    simple_script_controller.py 旧固定動作ノード
  config/semantic_places.yaml  過去の場所辞書
```

`main` の最小デモには Nav2 / YOLO を必要としません。以前の Nav2 / カメラ実験は `lv2_nav2_yolo` ブランチにあります。VM の未コミット変更は別途バックアップ済みで、上書きしていません。このデモは意味的場所への実ナビゲーションを主張しません。

## 1. 準備とビルド

ROS 2 Jazzy、Gazebo Sim 8、TurtleBot3 のセットアップ済み Ubuntu を対象とします。Humble や Gazebo Classic のコマンドを混用しないでください。
既存環境が不足する場合のみ（この apt 手順自体はクリーン OS で未検証）：

```bash
sudo apt update
sudo apt install python3-venv python3-colcon-common-extensions python3-yaml \
  ros-jazzy-turtlebot3-gazebo ros-jazzy-ros-gz
```

リポジトリルートで：

```bash
bash scripts/setup.sh
```

`venv` はシステム ROS Python を参照します。システム Python への pip インストールは行いません。Google SDK は `google-genai==2.34.0`。SDK の使い方は [Google 公式資料](https://googleapis.github.io/python-genai/) を参照。

## 2. シミュレーションを起動

**Ubuntu デスクトップ内の端末 A** でリポジトリルートから：

```bash
bash scripts/sim.sh
```

UTM で Ogre2 がクラッシュしたため、Ogre とソフトウェア描画を使います。Ubuntu のデスクトップ端末なら DISPLAY と XAUTHORITY は通常既に設定されています。SSH の場合は、実際のデスクトップセッションの値が必要です。値を推測せず確認してください。

## 3. AI を使わず接続を確認

別の端末 B、リポジトリルートで：

```bash
source /opt/ros/jazzy/setup.bash
source venv/bin/activate
source ros2_ws/install/setup.bash
python scripts/verify_motion.py
```

ロボットの周囲を空けて実行します。約 0.2 m 前進し右転して停止し、実測結果を出力します。これは固定動作テストです。

## 4. Gemini による自然言語デモ

端末 B で：

```bash
bash scripts/controller.sh
```

初回は API Key を非表示で入力します。設定済みの `GEMINI_API_KEY` / `GOOGLE_API_KEY`、またはローカルの `~/.config/pai/gemini_api_key` も使用できます。Key はコード、Git、スクリーン録画に含めません。Key ファイルを使う場合は権限を 600 にしてください。

入力例：

```text
0.2メートル前進して、右に90度回転して、停止してください
```

生成 JSON と Gazebo の動きを確認します。前進は 0.5 m 以下、旋回は 180 度以下、計画は最大 10 ステップ。末尾に STOP を補います。形式不正・範囲外・`GO_TO_PLACE` は実行前に拒否します。`q` または Ctrl+C で終了します。

同じ指示の API → 制御 → 里程計を自動確認する場合、Key を環境変数に設定した端末で `python scripts/verify_motion.py --gemini` を実行できます。固定計画版と同じ距離・角度を判定します。

モデルの初期値は既存プロジェクト由来の `gemini-2.5-flash`。利用可能なモデルはアカウントと時期で変わります。必要なら `GEMINI_MODEL` に利用可能なモデル ID を設定します。API エラー時は Key、モデル利用可否、ネットワーク、割当量を確認してください。

旧 Twist bridge の環境では `-p stamped_cmd_vel:=false` を ROS 引数として渡せます。ただし Humble での動作は今回未検証です。

## 停止と制約

まず制御端末で Ctrl+C。別端末からも停止を送れます：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/TwistStamped '{twist: {linear: {x: 0.0}, angular: {z: 0.0}}}'
```

動作ノードが生きていると指令を再発行するため、停止コマンドだけに依存しないでください。同時に teleop / Nav2 など別の速度発行ノードを動かさないでください。

距離と角度は速度×時間の近似です。障害物回避・画像認識・自己位置推定・実機用の安全機構はありません。シミュレーションの空いた場所で実行してください。

## 提出用

1. 実際の Gemini 解析から動作までの確認は完了。撮影時にも成功ログを残す。
2. 1〜3 分の動画で自然言語入力、生成 JSON、ロボット動作、停止を見せる。Key は映さない。
3. [REPORT.md](REPORT.md) の未検証箇所を実測結果で更新する。
4. ソースの差分と敏感情報を確認してから GitHub に反映する。動画、venv、ビルド成果物、API Key はコミットしない。
