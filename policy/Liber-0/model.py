"""Liber-0 policy adapter."""
import importlib
import os
from pathlib import Path
import sys

os.environ["TORCH_ALLOW_TF32_CUBLAS_OVERRIDE"] = "0"

import numpy as np
import torch
from XPolicyLab.model_template import ModelTemplate
from XPolicyLab.utils.checkpoint_resolver import resolve_checkpoint_root
from XPolicyLab.utils.process_data import get_robot_action_dim_info, pack_robot_state, unpack_robot_state


def _required_path(config, key):
    if not config.get(key):
        raise ValueError(f"Missing required path: {key}")
    path = Path(config[key]).expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(path)
    return path


def _load_policy(config, checkpoint_root):
    runtime_root = _required_path(config, "runtime_root")
    os.environ["PIXELWAM_MODEL_PATH"] = str(_required_path(config, "model_assets_path"))
    os.environ["PIXELWAM_SOURCE_PATH"] = str(_required_path(config, "backend_source_path"))
    # Explicit upstream runtime paths, not XPolicyLab package roots.
    sys.path.insert(0, str(runtime_root / "wam/model"))
    sys.path.insert(0, str(runtime_root))
    serving = importlib.import_module("pixelwam.serving")
    if Path(serving.__file__).resolve() != runtime_root / "wam/model/pixelwam/serving.py":
        raise RuntimeError(f"Wrong runtime imported: {serving.__file__}")
    return serving.Policy(
        checkpoint_path=str(checkpoint_root / config["weights_file"]),
        config_path=str(checkpoint_root / "config.yaml"),
        dataset_stats_path=str(checkpoint_root / "dataset_stats.json"),
        device=config["device"], port=0, model_dtype=torch.bfloat16,
        action_horizon=None, robotwin_camera_layout=None,
        num_inference_steps=int(config["num_inference_steps"]),
        seed=int(config["seed"]), rand_device="cpu", inference_compile=False,
    )


def _rgb(value):
    image = np.asarray(value)
    if image.ndim != 3 or image.shape[-1] != 3 or image.dtype != np.uint8:
        raise ValueError(f"Expected decoded uint8 HWC RGB, got {image.shape}/{image.dtype}")
    return image


class Model(ModelTemplate):
    def __init__(self, model_cfg):
        self.model_cfg = dict(model_cfg)
        self.action_type = self.model_cfg["action_type"]
        self.env_cfg_type = self.model_cfg["env_cfg_type"]
        if self.action_type != "joint" or self.env_cfg_type != "arx_x5":
            raise ValueError("This checkpoint supports arx_x5 joint control only.")
        if self.model_cfg["eval_batch"]:
            raise ValueError("Use eval_batch=false.")
        torch.set_float32_matmul_precision("highest")
        torch.backends.cuda.matmul.allow_tf32 = False
        print("[Liber-0 runtime]", sys.executable, torch.__version__, torch.version.cuda,
              os.environ["TORCH_ALLOW_TF32_CUBLAS_OVERRIDE"],
              torch.get_float32_matmul_precision(), torch.backends.cuda.matmul.allow_tf32, flush=True)
        assert os.environ["TORCH_ALLOW_TF32_CUBLAS_OVERRIDE"] == "0"
        assert torch.get_float32_matmul_precision() == "highest"
        assert not torch.backends.cuda.matmul.allow_tf32
        assert tuple(map(int, torch.__version__.split("+")[0].split(".")[:2])) >= (2, 11)
        if self.model_cfg["device"].startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError("A CUDA device is required; CPU inference is not selected automatically.")
        self.robot_action_dim_info = get_robot_action_dim_info(self.env_cfg_type)
        self.action_dim = sum(self.robot_action_dim_info["arm_dim"]) + sum(self.robot_action_dim_info["ee_dim"])
        policy_dir = Path(__file__).resolve().parent
        checkpoint_root = resolve_checkpoint_root(self.model_cfg, policy_dir / "checkpoints")
        self.policy = _load_policy(self.model_cfg, checkpoint_root)
        self.model = self.policy.model
        if self.model.stack != "hidream_o1" or self.policy.robotwin_camera_layout != "four_grid_288x384":
            raise ValueError("Incompatible checkpoint.")
        if self.model.dit.action_dim != self.action_dim:
            raise ValueError("Checkpoint action dimension does not match the registered robot.")
        if not self.model.action_expert_stability_fp32:
            raise ValueError("Checkpoint requires Action FP32.")
        self.replan_steps = int(self.model_cfg["replan_steps"])
        if not 0 < self.replan_steps <= self.policy.action_horizon:
            raise ValueError("replan_steps must be positive and no larger than the trained horizon.")
        self.seed = int(self.model_cfg["seed"])
        self.reset()

    def update_obs(self, obs):
        vision = obs["vision"]
        head = _rgb(vision["cam_head"]["color"])
        if self.episode_cue is None:
            self.episode_cue = head.copy()
        instruction = obs.get("instruction", obs.get("instructions"))
        if isinstance(instruction, (list, tuple)):
            if len(instruction) != 1:
                raise ValueError("Expected one instruction.")
            instruction = instruction[0]
        if not isinstance(instruction, str) or not instruction.strip():
            raise ValueError("An explicit task instruction is required.")
        self.instruction = instruction
        self.observation = {
            "head_camera_rgb": head,
            "left_camera_rgb": _rgb(vision["cam_left_wrist"]["color"]),
            "right_camera_rgb": _rgb(vision["cam_right_wrist"]["color"]),
            "visual_cue_rgb": self.episode_cue,
            "joint_action_vector": np.asarray(pack_robot_state(
                obs, self.action_type, self.robot_action_dim_info,
            ), dtype=np.float32),
        }

    def get_action(self):
        if self.observation is None:
            raise RuntimeError("Call update_obs after reset before requesting actions.")
        # Only the public run seed and local replan counter determine noise.
        actions = np.asarray(self.policy._infer_action_chunk(
            self.observation, self.instruction, request_seed=self.seed + self.replan_index,
        ), dtype=np.float32)
        if actions.shape != (self.policy.action_horizon, self.action_dim) or not np.isfinite(actions).all():
            raise ValueError(f"Invalid predicted action chunk: {actions.shape}")
        self.replan_index += 1
        return unpack_robot_state(actions[:self.replan_steps], self.action_type, self.robot_action_dim_info)

    def update_obs_batch(self, obs_list):
        raise NotImplementedError("Use eval_batch=false and one environment per policy server.")

    def get_action_batch(self, env_idx_list=None):
        raise NotImplementedError("Use eval_batch=false and one environment per policy server.")

    def reset(self):
        self.observation = None
        self.instruction = None
        self.episode_cue = None
        self.replan_index = 0
