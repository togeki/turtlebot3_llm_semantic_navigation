# 8. LV2 – Nav2 + YOLO 視覚セマンティックナビゲーション

## 8.1 目標

  カメラと YOLO を使って環境中の物体（箱・ドアなど）を検出し，  
  LLM が `GO_TO_OBJECT` のようなアクションを生成して，Nav2 で TurtleBot3 を移動させる。

---

## 8.2 LV2 で追加したコンポーネント

- `worlds/semantic_room.world`  
  - いくつかの箱が置かれた簡単な部屋の Gazebo ワールド  
  - YOLO が検出しやすいように，色付きのオブジェクトを配置

- `urdf/tb3_with_cam.urdf.xacro`  
  - TurtleBot3 に頭上カメラ（`camera_link`）を追加した URDF  
  - Gazebo の camera plugin により `/camera/image_raw` を Publish

- `tb3_llm_controller/nodes/yolo_detector.py`（予定）  
  - `/camera/image_raw` を Subscribe  
  - YOLO で物体検出を行い，`/detected_objects` トピックとして Publish

- `tb3_llm_controller/nodes/nav2_client.py`（予定）  
  - Nav2 の `NavigateToPose` Action をラップするクライアントノード  
  - サービスや簡単な API で「(x, y, theta) に移動して」と要求できる

---
