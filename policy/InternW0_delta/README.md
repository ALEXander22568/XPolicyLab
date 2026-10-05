# InternW0_delta

**Contributor:** Xingyu Miao | **Paper:** InternW0-Δ | **arXiv:** [2609.31394](https://arxiv.org/abs/2609.31394) | **Original code:** [InternRobotics/InternW0-Delta](https://github.com/InternRobotics/InternW0-Delta)

This adapter reproduces the RoboDojo evaluation of InternW0_delta for `env_cfg_type=arx_x5` with `action_type=joint`, using a 32-step action horizon, replanning after 10 executed actions, and 10 denoising steps. Evaluation still uses the lightweight vendored `wam_runtime/` by default; training and data processing are wired to the full official InternW0-Delta source checkout because the training stack requires `wam.datasets`, Hydra configs, Accelerate/DeepSpeed launch scripts, and post-training tools that are not part of the eval-only runtime.

Shared conventions — argument meanings, checkpoint naming, split-machine deployment, `EVAL_ENV_TYPE` — are documented in the [XPolicyLab README](../../README.md). Official results: [RoboDojo LeaderBoard](https://robodojo-benchmark.com/LeaderBoard).

## Installation

```bash
cd policy/InternW0_delta
bash install.sh internw0-delta
conda activate internw0-delta
```

For training/data processing, clone the official source into this policy
directory, or point `INTERNW0_DELTA_ROOT` at an external checkout before
running `install.sh`:

```bash
cd policy/InternW0_delta
bash sync_upstream.sh
bash install.sh internw0-delta
```

`install.sh` also installs a local C/C++ toolchain because RynnBrain's
FLA/Triton kernels compile a small runtime helper during their first forward
pass. No system-wide compiler installation is required. The tested policy
runtime is Python 3.11, PyTorch 2.10.0 with CUDA 12.8, Transformers 5.13.0,
and BF16 inference; the install script pins these model-sensitive packages.
When the full official source exists, `install.sh` installs it with
`[train,modelscope]` extras; otherwise it installs only `wam_runtime/` for
evaluation.

## Data Processing

`process_data.sh` prepares the official RoboDojo post-training layout:

```bash
cd policy/InternW0_delta
bash process_data.sh RoboDojo <task_name> arx_x5 joint [expert_data_num] [raw_task_dirs]
```

The InternW0-Delta post-training loader reads **LeRobot v2.1**
(`meta/tasks.jsonl`, `meta/episodes.jsonl`, one parquet per episode), so the
wrapper calls the official converter `scripts/transform_lerobot_v21_format.py`
at 480x640 — offline decoding therefore goes through `decode_image_bit`, and
the keys (`observation.images.{cam_high,cam_left_wrist,cam_right_wrist}`,
`observation.state`, `action`, 14D) are the official ones that
`configs/data/robodojo.yaml` expects. No custom converter is involved. The
dataset is written to
`policy/InternW0_delta/data/<bench>-<ckpt>-<env>-<action>/{meta,data,videos}`
(override with `ROBODOJO_DATA_ROOT`); an already complete dataset there is
reused, not regenerated.

The converter imports `lerobot.datasets.lerobot_dataset` from a LeRobot
release that still writes codebase v2.1. That package is not part of the
`internw0-delta` environment (the upstream code vendors its own reader), so
point `INTERNW0_CONVERT_PYTHON` at a Python that has it, e.g.
`INTERNW0_CONVERT_PYTHON=/path/to/lerobot-env/bin/python`.

Inputs are read from the parent workspace's raw-data layout,
`data/RoboDojo/<task>/arx_x5/data/*.hdf5`. `raw_task_dirs` defaults to
`<task_name>` and may be comma-separated to merge several task folders;
`expert_data_num` caps the episodes per task.

The wrapper then builds the official text-embedding cache
(`tools/text_cache.py task=robodojo`) with the local Wan2.2 UMT5 encoder from
`download_assets.sh`, writing to `.cache/internw0/text_embed/robodojo`. Set
`INTERNW0_SKIP_TEXT_CACHE=1` to skip it, and `INTERNW0_DELTA_ROOT` if the
official checkout is not `policy/InternW0_delta/InternW0-Delta`.

## Training

Post-training is launched through the official InternW0-Delta recipe:

```bash
cd policy/InternW0_delta
bash train.sh RoboDojo <task_name> arx_x5 joint <seed> <gpu_id> [num_gpus]
```

`train.sh` writes artifacts to the XPolicyLab-standard run directory:

```text
policy/InternW0_delta/checkpoints/RoboDojo-<task_name>-arx_x5-joint-<seed>/
  checkpoints/weights/slot_*.pt
  checkpoints/weights/weights_manifest.json
  checkpoints/state/latest/
  config.yaml
```

Before training, run `download_assets.sh` and place the official
[`InternW0-Delta-Base`](https://huggingface.co/InternRobotics/InternW0-Delta-Base)
weights at `policy/InternW0_delta/checkpoints/pretrain.pt` (or set
`WAM_PRETRAIN_CHECKPOINT`). The wrapper reuses the evaluation assets — Wan2.2
VAE in its original format and RynnBrain from `assets/` — by passing
`model.redirect_common_files=false` and the local `model_id` /
`tokenizer_model_id`, the same settings as the official RoboDojo evaluation
config; `DIFFSYNTH_SKIP_DOWNLOAD=true` makes a missing file fail instead of
being downloaded elsewhere. The video DiT is not loaded from Wan2.2
(`skip_dit_load_from_pretrain=true`); it comes from `pretrain.pt`.

It auto-runs `process_data.sh` when the dataset is missing. Extra Hydra
overrides can be appended through `INTERNW0_TRAIN_OVERRIDES`; keep model
architecture keys unchanged, because evaluation rebuilds the model from
`config/eval_model.yaml`:

```bash
INTERNW0_TRAIN_OVERRIDES="max_steps=1000 save_every=500" \
  bash train.sh RoboDojo stack_bowls arx_x5 joint 0 0
```

## Evaluation

```bash
bash eval.sh <bench_name> <task_name> <ckpt_name> <env_cfg_type> <action_type> <seed> \
  <policy_gpu_id> <env_gpu_id> <policy_conda_env> <eval_env_conda_env>

# Example: evaluate checkpoints/robodojo.pt on stack_bowls
bash eval.sh RoboDojo stack_bowls robodojo arx_x5 joint 0 0 0 internw0-delta <eval_env_conda_env>
```

The checkpoint comes from `checkpoint_path` in `deploy.yml` (or
`WAM_CHECKPOINT_PATH`) for released weights. For locally trained checkpoints,
`setup_eval_policy_server.sh` also resolves the latest official
`checkpoints/weights/slot_*.pt` under
`checkpoints/<bench>-<ckpt>-<env>-<action>-<seed>/` (pass the short task
name or the full run-directory name as `ckpt_name`), using the `latest` entry
of `weights_manifest.json`, else the newest slot by modification time. A run
directory without weights is an error rather than a silent fallback to
`robodojo.pt`. Evaluation keeps `config/dataset_stats.json`, which is
identical to the upstream `assets/stats/robodojo.json` used by training. For the offline wiring check without
model weights, run
`EVAL_ENV_TYPE=debug WAM_ALLOW_DUMMY_POLICY=true bash eval.sh ...` with the
same arguments; leave `EVAL_ENV_TYPE` unset or set `EVAL_ENV_TYPE=sim` for
RoboDojo simulation. For split-machine deployment via
`setup_eval_policy_server.sh` / `setup_eval_env_client.sh`, follow the
[Deployment Flow](../../README.md#-deployment-flow).

## Model Assets

Model weights are deliberately excluded from Git and from source archives.
The evaluator must prepare these three artifacts:

| Artifact | Upstream ID / source | Purpose | Local path relative to this directory |
| --- | --- | --- | --- |
| Wan2.2 | `Wan-AI/Wan2.2-TI2V-5B` | WAM video backbone, VAE and tokenizer resources | `assets/Wan-AI/Wan2.2-TI2V-5B/` |
| RynnBrain | `Alibaba-DAMO-Academy/RynnBrain1.1-2B` | visual-language understanding encoder | `assets/Alibaba-DAMO-Academy/RynnBrain1.1-2B/` |
| InternW0_delta checkpoint | [`InternRobotics/InternW0-Delta-RoboDojo`](https://huggingface.co/InternRobotics/InternW0-Delta-RoboDojo) | RoboDojo evaluation weights, file `robodojo.pt` | `checkpoints/robodojo.pt` |

Download the two public base models from ModelScope:

```bash
bash download_assets.sh
```

For Wan2.2 the script downloads only the VAE, UMT5 encoder and tokenizer files
used by evaluation; the full Video-DiT snapshot is unnecessary because
`robodojo.pt` already contains that expert. RynnBrain is downloaded as its full
ModelScope snapshot because Transformers loads its processor, tokenizer,
configuration, chat template, and model weights from that local directory.

This command writes only below `assets/`. Review and comply with the upstream
model licenses before downloading. To use an already populated model store,
pass its destination root instead:

```bash
bash download_assets.sh /path/to/local/assets
```

Download the evaluation checkpoint from Hugging Face and verify its hash:

```bash
bash download_checkpoint.sh
```

A local file or HTTPS URL can be passed instead; it is copied to
`checkpoints/robodojo.pt` and checked against the same SHA256.

The checked-in z-score statistics are in `config/dataset_stats.json`. The
exact expected artifact paths and hashes are recorded in
`config/artifacts.lock.json`.

## Configuration

All model locations are resolved from this policy directory by default.
Adapter-specific `deploy.yml` keys:

- Artifact paths: `checkpoint_path`, `base_model_dir`, `vlm_model_path`,
  `dataset_stats_path`, `train_config_path`.
- Inference contract of the checkpoint (keep the defaults to reproduce the
  reported result): `device`, `mixed_precision`, `action_horizon`,
  `replan_steps`, `num_inference_steps`, `action_hz`, `text_cfg_scale`,
  `negative_prompt`, `rand_device`, `tiled`.
- Diagnostics: `timing_enabled`, `default_instruction` (used when an
  observation carries no instruction), `allow_dummy_policy`.

`setup_eval_policy_server.sh` reads these environment variables:

| Environment variable | Override |
| --- | --- |
| `WAM_CHECKPOINT_PATH` | post-trained `robodojo.pt` file |
| `WAM_DATASET_STATS_PATH` | z-score statistics JSON |
| `WAM_EVAL_CONFIG_PATH` | evaluation model configuration |
| `WAM_WAN22_PATH` | local Wan2.2 directory |
| `WAM_RYNNBRAIN_PATH` | local RynnBrain directory |
| `WAM_ALLOW_DUMMY_POLICY=true` | debug wiring only; skips all real weights |

Evaluation uses the checkpoint's z-score `global_mean` / `global_std`
statistics, RGB input without training-time color jitter, discrete Action
RoPE, physical-time RoPE disabled, fan-in calibration disabled, and the
recent-KV-cache mask fix.

## Notes

- Batched evaluation (`eval_batch: true`): the policy is stateful, so the
  adapter keeps one WAM session (memory frames, pending actions, step counter)
  per `env_idx`, and every environment replans through the same single-sample
  inference path as `eval_batch: false`; GPU inference runs one environment
  at a time. Batched observations must carry `env_idx`, as the RoboDojo and
  debug environment clients do.
- Training/data wrappers require the public official repository checkout; the
  eval-only vendored runtime remains intentionally small.
- Reference result: a self-run, single-environment evaluation over 54 tasks
  and 6,300 episodes completed 1,444 successful episodes — 22.92% success rate
  and 30.35 mean score.
