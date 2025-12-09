# 10. LV2 – Nav2 Integration (Semantic Map)

## 10-1. 概要

LV1.5 までで，LLM プランナーとセマンティックマップ（semantic_places.yaml）を使って  
「GO_TO_PLACE(window)」のような高レベルな行動計画を生成できるようになった。

本ブランチ LV2 では，その出力を **Nav2 の自律移動** につなげること，  
さらに将来的な **YOLO による物体認識連携** の準備を行う。

---

## 10-2. 目標

- ユーザが自然言語で目的地を指示（例：「窓のところへ行って」→ `window`）
- LLM が `GO_TO_PLACE(window)` のような plan を生成
- セマンティック地点情報 (semantic_places.yaml) をもとに Nav2 が TurtleBot3 を自律移動

今後は YOLO によって環境中の物体（窓・ドアなど）を識別し，  
`GO_TO_OBJECT("window")` のようなアクションまで拡張することを目標とする。

---

## 10-3. LV2 で追加・更新したコンポーネント

- `config/semantic_places.yaml`（既存）
  - セマンティック地点（`window`, `door` など）の座標 (x, y, theta) を定義
  - LLM が生成した `GO_TO_PLACE` を Nav2 用の具体的な位置情報に変換するための辞書

- `tb3_llm_controller/nodes/llm_controller.py`（更新）
  - ユーザ入力 → LLM → 行動計画（plan）
  - plan 内の `GO_TO_PLACE` の `target` を `/llm_place` に Publish

- `tb3_llm_controller/nodes/semantic_place_bridge.py`
  - `/llm_place` で受け取った地点名を `semantic_places.yaml` から検索
  - 例：`"window"` → `{x: ..., y: ..., yaw: ...}` → `/llm_nav_json` に Publish
  - セマンティック層 → ナビゲーション層の橋渡し

- `tb3_llm_controller/nodes/llm_nav_bridge.py`
  - JSON を Nav2 が扱える `PoseStamped` に変換
  - `/go_to_pose` に送信し，Nav2 の NavigateToPose Action をトリガーする

- `tb3_llm_controller/nodes/nav2_client.py`
  - Nav2 の NavigateToPose Action を wrap
  - 受け取った Pose へロボットを移動させる最終実行部

---

# 10-4. LV2 全体アーキテクチャ

1. User Input（自然言語）
2. LLM `get_plan`  
   → `plan = [{"action": "GO_TO_PLACE", "target": "window"}]`
3. `llm_controller.py`  
   → `"window"` を `/llm_place` に Publish
4. `semantic_place_bridge.py`  
   → `"window"` → `{x, y, yaw}` → `/llm_nav_json`
5. `llm_nav_bridge.py`  
   → JSON → `PoseStamped` → `/go_to_pose`
6. `nav2_client.py`  
   → Nav2 NavigateToPose Action を実行

結果：ユーザは「窓へ行って」と言うだけでロボットは自律的に移動する

---

## 10-5. 10-4 使い方 (簡易メモ)

前提：

- LV1 / LV1.5 のセットアップが完了していること
- Nav2 が単独で動作し，事前に地図が作成されていること
- LLM API キーなどが設定済みであること

起動イメージ（例）：

1. Nav2 スタックを起動
2. 本パッケージのノード群（`llm_controller`, `semantic_place_bridge`, `llm_nav_bridge`, `nav2_client` など）を起動
3. ターミナル or GUI から自然言語で指示  
   例：`「窓のところへ行って」` / `go to the window`

※ 実際の launch ファイル・コマンドは後で整理して追記予定。





