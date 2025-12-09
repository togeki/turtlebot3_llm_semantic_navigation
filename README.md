# 10. LV2 – Nav2 + YOLO 視覚セマンティックナビゲーション

## 10-1 目標

Nav2 と LLM を連携させることで，

ユーザが自然言語で目的地を指示
（例：「窓のところへ行って」→ window）

セマンティック地点情報をもとに
Nav2 が TurtleBot3 を自律移動

できるシステムを実現する。

今後は YOLO によって環境中の物体（窓・ドアなど）を識別し，
GO_TO_OBJECT のようなアクションまで拡張する。

---

## 10-2 LV2 で追加したコンポーネント

 - config/semantic_places.yaml

セマンティック地点（window, door）の座標 (x, y, theta) を定義
LLM が生成した GO_TO_PLACE を Nav2 宛ての明確な位置情報に変換するための辞書

 - tb3_llm_controller/nodes/llm_controller.py（更新済）

```text
ユーザ入力 → LLM → 行動計画（plan）
    ↓
plan 内の GO_TO_PLACE target を /llm_place に Publish
```

これにより LLM が「どこへ行くべきか」を決め，
移動自体は Nav2 が担当する明確な役割分離が実現した。

 - tb3_llm_controller/nodes/semantic_place_bridge.py（実装済）

/llm_place で受け取った地点名を YAML から検索し，

```yaml
"window" → {x: ..., y: ..., yaw: ...} → JSON
```

として /llm_nav_json に Publish
セマンティック層 → ナビゲーション層の橋渡しを行う

 - tb3_llm_controller/nodes/nav2_client.py（実装済）

Nav2 の NavigateToPose Action を wrap し，
受け取った Pose へロボットを移動させる最終実行部

---

# 10-3. LV2 全体アーキテクチャ

```bash
User Input（自然言語）
        ↓
LLM (get_plan)
        ↓
plan: [{"action": "GO_TO_PLACE", "target": "window"}]
        ↓
llm_controller.py
   publish "window" → /llm_place
        ↓
semantic_place_bridge.py
   "window" → {x,y,yaw} → /llm_nav_json
        ↓
llm_nav_bridge.py
   JSON → PoseStamped → /go_to_pose
        ↓
nav2_client.py
   Nav2 自律走行
```

結果：ユーザは「窓へ行って」と言うだけでロボットは自律的に移動する

---
