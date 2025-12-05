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
あなたは移動ロボット（TurtleBot3）の行動計画を立てるプランナーです。
ユーザから与えられた【日本語の移動指示文】を，以下の JSON フォーマットに変換してください。

利用できるアクションは次の 4 つだけです：
- FORWARD(distance)  // 前進。単位：メートル (m)
- TURN_LEFT(angle)   // 左回転。単位：度 (degree)
- TURN_RIGHT(angle)  // 右回転。単位：度 (degree)
- STOP               // 停止。value は 0 とする

出力フォーマット（例）：
{
  "plan": [
    {"action": "FORWARD", "value": 0.5},
    {"action": "TURN_LEFT", "value": 90},
    {"action": "STOP", "value": 0}
  ]
}

必ず次のルールを守ってください：
- 出力は JSON のみ。余計な文章や説明を書かない。
- action は必ず大文字（FORWARD / TURN_LEFT / TURN_RIGHT / STOP）。
- value は数値（float）で出力する。
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

    plan = data.get("plan", [])
    if not isinstance(plan, list):
        raise ValueError("JSON 中の 'plan' がリストではありません: " + str(data))

    return plan


if __name__ == "__main__":
    cmd = input("日本語でロボットへの指示を入力してください：\n> ")
    plan = get_plan(cmd)
    print("生成された行動計画:")
    print(json.dumps({"plan": plan}, ensure_ascii=False, indent=2))

