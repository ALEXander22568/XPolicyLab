# Liber-0

RoboDojo evaluation policy for `arx_x5` with joint actions.

Shared conventions — argument meanings, checkpoint naming, split-machine deployment, `EVAL_ENV_TYPE` — are documented in the [XPolicyLab README](../../README.md). Official results: [RoboDojo LeaderBoard](https://robodojo-benchmark.com/LeaderBoard).

## Installation

Requires PyTorch 2.11 or later and the separately supplied inference source.
Component requirements and notices: [NOTICE.md](NOTICE.md).

```bash
bash install.sh /path/to/env
export LIBER0_RUNTIME_ROOT=/path/to/runtime
export LIBER0_MODEL_PATH=/path/to/base-model-assets
export LIBER0_SOURCE_PATH=/path/to/backend-source
export TORCH_ALLOW_TF32_CUBLAS_OVERRIDE=0
```

## Data Processing

Not required for evaluation.

## Training

This adapter is evaluation-only.

## Evaluation

The checkpoint directory must contain `model.pt`, `config.yaml` and
`dataset_stats.json`.

```bash
python download_checkpoint.py --repo-id zxw1810/PixelWAM-RoboDojo \
  --revision COMMIT_SHA --destination ./checkpoint

EVAL_ENV_TYPE=sim bash eval.sh RoboDojo stack_bowls /path/to/checkpoint \
  arx_x5 joint 0 0 0 /path/to/env RoboDojo
```

For interface checks, use `EVAL_ENV_TYPE=debug`; repeat with
`DEBUG_OBS_ENCODED=1` to test encoded observations. One environment per policy
server is supported; use `eval_batch=false`.
