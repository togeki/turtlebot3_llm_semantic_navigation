CREATE TABLE IF NOT EXISTS runs (
  run_id          UUID PRIMARY KEY,
  started_at      TIMESTAMPTZ NOT NULL,
  ended_at        TIMESTAMPTZ,
  ros_distro      TEXT DEFAULT 'humble',
  map_name        TEXT,
  world_name      TEXT,
  nav2_config     TEXT,
  user_text       TEXT,
  llm_plan_json   JSONB,
  target_place    TEXT,
  target_pose     JSONB,
  result_status   TEXT,
  result_detail   JSONB
);

CREATE TABLE IF NOT EXISTS events (
  id        BIGSERIAL PRIMARY KEY,
  run_id    UUID REFERENCES runs(run_id) ON DELETE CASCADE,
  ts        TIMESTAMPTZ NOT NULL,
  type      TEXT NOT NULL,
  payload   JSONB
);
CREATE INDEX IF NOT EXISTS idx_events_run_ts ON events(run_id, ts);

CREATE TABLE IF NOT EXISTS timeseries_messages (
  id        BIGSERIAL PRIMARY KEY,
  run_id    UUID REFERENCES runs(run_id) ON DELETE CASCADE,
  ts        TIMESTAMPTZ NOT NULL,
  topic     TEXT NOT NULL,
  frame_id  TEXT,
  payload   JSONB NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_ts_run_ts ON timeseries_messages(run_id, ts);
CREATE INDEX IF NOT EXISTS idx_ts_run_topic_ts ON timeseries_messages(run_id, topic, ts);
