# tb3_llm_controller/gemini_planner.py

import os
import json
from google import genai
from google.genai import types

# 環境変数から API キーを取得
API_KEY = os.getenv("GEMINI_API_KEY")
if not API_KEY:
    raise RuntimeError("環境変数 GEMINI_API_KEY が設定されていません。export GEMINI_API_KEY=... してください。")

# Gemini client is initialized when needed.

# 利用するモデル（あなたの list_models にあったやつ）
MODEL_NAME = "gemini-2.5-flash"


SYSTEM_PROMPT = """
あなたは屋内巡回ロボット TurtleBot3 の行動計画エージェントです。
日本語の指示を解釈し、JSON 配列のみを出力してください。

利用できる巡回地点は次の4か所です。
point_a, point_b, point_c, point_e

各地点は実際の屋内地図に登録された到達可能な巡回地点です。
地点名から座標を推測しないでください。

利用可能な行動:
- GO_TO_PLACE: {"action":"GO_TO_PLACE","target":"point_a"}
- FORWARD: {"action":"FORWARD","value":0.2}
- TURN_LEFT: {"action":"TURN_LEFT","value":90}
- TURN_RIGHT: {"action":"TURN_RIGHT","value":90}
- STOP: {"action":"STOP","value":0}

巡回指示の場合:
- 「家を巡回して」「4か所を巡回して」などの指示では、
  point_c → point_a → point_e → point_b
  の順に GO_TO_PLACE を4個出力してください。
- 指定された地点だけを訪問する指示では、指定順に出力してください。
- 地点間の移動方法は Nav2 が決定するため、FORWARD や TURN を追加しないでください。
- すべての巡回計画の最後に STOP を追加してください。
- 登録されていない場所への移動を要求された場合は STOP のみ出力してください。
- 出力はJSON配列のみ。説明文やMarkdownは不要です。

例:
入力: 家の中の4か所を巡回してください
出力:
[
 {"action":"GO_TO_PLACE","target":"point_c"},
 {"action":"GO_TO_PLACE","target":"point_a"},
 {"action":"GO_TO_PLACE","target":"point_e"},
 {"action":"GO_TO_PLACE","target":"point_b"},
 {"action":"STOP","value":0}
]
"""



def call_gemini(command: str) -> str:
    """日本語コマンドから JSON 文字列（plan を含む）を生成する。"""


    prompt = SYSTEM_PROMPT + "\n\nユーザの指示文：\n" + command

    with genai.Client(
        api_key=API_KEY,
        http_options=types.HttpOptions(timeout=30000)
    ) as client:
        response = client.models.generate_content(
            model=MODEL_NAME,
            contents=prompt,
            config=types.GenerateContentConfig(
                response_mime_type="application/json",
                temperature=0
            )
        )

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

