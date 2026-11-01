# tb3_llm_controller/gemini_planner.py

import os
import json
from google import genai
from google.genai import types

# Keep credentials outside source control. Read them only when calling the API.
MODEL_NAME = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

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

重要：
- FORWARD は value に距離（m、0 より大きく 0.5 以下）を出力してください。指定がなければ 0.2。
- TURN_LEFT / TURN_RIGHT は value に角度（度、0 より大きく 180 以下）を出力してください。指定がなければ 90。
- STOP は value: 0。今回のデモでは GO_TO_PLACE は使用しないでください。
- JSON以外の文章は出力しないでください。

入力例：
「0.2メートル前進して右に90度回転して停止して」
出力例：
[
  {"action": "FORWARD", "value": 0.2},
  {"action": "TURN_RIGHT", "value": 90},
  {"action": "STOP", "value": 0}
]

それでは、ユーザ入力に対する行動計画を生成してください。
"""


def call_gemini(command: str) -> str:
    """日本語コマンドから JSON 文字列（plan を含む）を生成する。"""
    api_key = os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")
    if not api_key:
        raise RuntimeError("GEMINI_API_KEY を設定してください。")
    prompt = SYSTEM_PROMPT + "\n\nユーザの指示文：\n" + command
    with genai.Client(api_key=api_key, http_options=types.HttpOptions(timeout=30000)) as client:
        response = client.models.generate_content(
            model=MODEL_NAME,
            contents=prompt,
            config=types.GenerateContentConfig(response_mime_type="application/json", temperature=0),
        )
    if not response.text:
        raise ValueError("Gemini returned no text")
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

    return validate_plan(plan)

def validate_plan(plan):
    """Reject unsupported or unbounded commands before any motion starts."""
    import math
    if not isinstance(plan, list) or not 1 <= len(plan) <= 10:
        raise ValueError("Plan must contain 1–10 steps")
    validated = []
    for step in plan:
        if not isinstance(step, dict):
            raise ValueError("Each step must be an object")
        action = step.get("action")
        if action not in ("FORWARD", "TURN_LEFT", "TURN_RIGHT", "STOP"):
            raise ValueError("Unsupported action: " + str(action))
        value = step.get("value")
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("A finite numeric value is required")
        limit = 0.5 if action == "FORWARD" else 180
        if (action == "STOP" and value != 0) or (action != "STOP" and not 0 < value <= limit):
            raise ValueError("Value outside demo limits")
        validated.append({"action": action, "value": float(value)})
    if validated[-1]["action"] != "STOP":
        validated.append({"action": "STOP", "value": 0.0})
    return validated


if __name__ == "__main__":
    cmd = input("日本語でロボットへの指示を入力してください：\n> ")
    plan = get_plan(cmd)
    print("生成された行動計画:")
    print(json.dumps({"plan": plan}, ensure_ascii=False, indent=2))

