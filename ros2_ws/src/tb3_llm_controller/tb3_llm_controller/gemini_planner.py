# tb3_llm_controller/gemini_planner.py

import os
import json
import google.generativeai as genai

# 環境変数から API キーを取得
API_KEY = os.getenv("GEMINI_API_KEY")
if not API_KEY:
    raise RuntimeError("環境変数 GEMINI_API_KEY が設定されていません。export GEMINI_API_KEY=... してください。")

genai.configure(api_key=API_KEY)

# 利用するモデル（あなたの list_models にあったやつ）
MODEL_NAME = "models/gemini-2.5-flash"


SYSTEM_PROMPT = """
あなたはロボットのための行動計画エージェントです。

入力は人間が話す日本語の指示です。
出力は JSON のリストで、各要素は 1 ステップの行動を表します。

行動(action)は次の2種類のみを使用してください：

1) 低レベル行動（LV1）
   - "FORWARD": まっすぐ進む
   - "TURN_LEFT": 左へ曲がる
   - "TURN_RIGHT": 右へ曲がる
   - "STOP": 止まる

2) 意味的移動（LV1.5 / LV2）
   - "GO_TO_PLACE": 目的地（"window", "door", "table", "water" など）
      例: { "action": "GO_TO_PLACE", "target": "window" }

重要：
- 「窓」「ドア」「机」などの語彙は、必ず target に英語の識別子
  ("window", "door", "table") として出力してください。
- 移動距離や角度は出力しないでください。ロボット側で処理します。
- JSON以外の文章は出力しないでください。

入力例：
「窓のところに行って止まって」
出力例：
[
  {"action": "GO_TO_PLACE", "target": "window"},
  {"action": "STOP"}
]

それでは、ユーザ入力に対する行動計画を生成してください。
"""


def call_gemini(command: str) -> str:
    """日本語コマンドから JSON 文字列（plan を含む）を生成する。"""
    model = genai.GenerativeModel(MODEL_NAME)

    prompt = SYSTEM_PROMPT + "\n\nユーザの指示文：\n" + command

    response = model.generate_content(prompt)

    # response.text には JSON 文字列または ```json ... ``` 形式が入ることが多い
    return response.text


def extract_json(text: str) -> dict:
    """```json ... ``` を含んでいても，純粋な JSON 部分だけを抜き出して dict にする。"""
    text = text.strip()

    # ```json や ``` のブロックを除去
    if text.startswith("```"):
        lines = text.splitlines()
        # 先頭の ```xxx を削除
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        # 末尾の ``` を削除
        if lines and lines[-1].strip().startswith("```"):
            lines = lines[:-1]
        text = "\n".join(lines).strip()

    data = json.loads(text)
    return data


def get_plan(command: str):
    """
    日本語の指示文 → {"plan": [...] } の plan 部分（list）を返す。
    例）[{"action": "FORWARD", "value": 0.5}, ...]
    """
    raw = call_gemini(command)
    data = extract_json(raw)

    # 新しいフォーマットに対応：
    # - data がそのままリストの場合 → それを plan とみなす
    # - data が {"plan": [...]} の場合 → 従来通り取り出す
    if isinstance(data, list):
        plan = data
    elif isinstance(data, dict):
        plan = data.get("plan", [])
    else:
        raise ValueError("JSON の形式が不明です: " + str(data))

    if not isinstance(plan, list):
        raise ValueError("最終的な plan がリストではありません: " + str(data))

    return plan

if __name__ == "__main__":
    cmd = input("日本語でロボットへの指示を入力してください：\n> ")
    plan = get_plan(cmd)
    print("生成された行動計画:")
    print(json.dumps({"plan": plan}, ensure_ascii=False, indent=2))

