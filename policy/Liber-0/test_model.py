"""CPU contract tests; the neural inference engine is explicitly mocked."""
import asyncio
import importlib
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yaml

adapter = importlib.import_module("XPolicyLab.policy.Liber-0.model")
from XPolicyLab.utils.process_data import decode_obs_images, encode_image_bit, pack_robot_state


class Engine:
    action_horizon = 32
    robotwin_camera_layout = "four_grid_288x384"
    model = SimpleNamespace(stack="hidream_o1", dit=SimpleNamespace(action_dim=14), action_expert_stability_fp32=True)

    def __init__(self):
        self.calls = []
        self.actions = np.tile(np.arange(14, dtype=np.float32), (32, 1))

    def _infer_action_chunk(self, observation, instruction, request_seed):
        self.calls.append((observation, instruction, request_seed))
        return self.actions.copy()


@pytest.fixture
def policy(monkeypatch, tmp_path):
    config = yaml.safe_load((Path(__file__).parent / "deploy.yml").read_text())
    config.update(device="cpu", action_type="joint", env_cfg_type="arx_x5", seed=0,
                  ckpt_name=str(tmp_path), bench_name="RoboDojo", task_name="unused")
    engine = Engine()
    monkeypatch.setattr(adapter, "_load_policy", lambda cfg, root: engine)
    monkeypatch.setattr(adapter, "get_robot_action_dim_info", lambda env: {"arm_dim": [6, 6], "ee_dim": [1, 1]})
    return adapter.Model(config)


def observation(value=10):
    return {
        "instruction": "Place the object in the box.",
        "vision": {name: {"color": np.full((24, 32, 3), value + i, dtype=np.uint8)}
                   for i, name in enumerate(("cam_head", "cam_left_wrist", "cam_right_wrist"))},
        "state": {"left_arm_joint_state": np.arange(6), "left_ee_joint_state": np.array([6]),
                  "right_arm_joint_state": np.arange(7, 13), "right_ee_joint_state": np.array([13])},
    }


def test_cue_is_copied_and_fixed_until_reset(policy):
    first = observation(10)
    policy.update_obs(first)
    first["vision"]["cam_head"]["color"][:] = 80
    policy.update_obs(observation(30))
    assert np.all(policy.observation["visual_cue_rgb"] == 10)
    assert np.all(policy.observation["head_camera_rgb"] == 30)
    policy.reset()
    policy.update_obs(observation(40))
    assert np.all(policy.observation["visual_cue_rgb"] == 40)


def test_joint_order_horizon_and_seed_sequence(policy):
    obs = observation()
    obs.update(task_name="not used", layout_id=123, _external_request_seed=999)
    policy.update_obs(obs)
    actions = policy.get_action()
    assert len(actions) == 16
    assert np.array_equal(policy.observation["joint_action_vector"], np.arange(14))
    for item in actions:
        assert np.array_equal(pack_robot_state({"state": item}, "joint", policy.robot_action_dim_info), np.arange(14))
    policy.get_action()
    assert [call[2] for call in policy.policy.calls] == [0, 1]
    policy.reset()
    policy.update_obs(observation())
    policy.get_action()
    assert policy.policy.calls[-1][2] == 0


def test_readonly_rgb_observations_are_supported(policy):
    obs = observation()
    for view in obs["vision"].values():
        view["color"].flags.writeable = False
    policy.update_obs(obs)
    assert policy.episode_cue.flags.writeable
    assert len(policy.get_action()) == 16


def test_server_decoded_encoded_rgb_has_no_extra_channel_swap(policy):
    obs = observation()
    colors = ((210, 20, 40), (20, 210, 40), (20, 40, 210))
    for i, (view, color) in enumerate(zip(obs["vision"].values(), colors)):
        image = np.full((24, 32, 3), color, dtype=np.uint8)
        encoded = encode_image_bit(image)
        view["color"] = np.asarray(bytearray(encoded), dtype=np.uint8) if i == 0 else encoded if i == 1 else image
    decoded = decode_obs_images(obs)
    policy.update_obs(decoded)
    for key, channel in (("head_camera_rgb", 0), ("left_camera_rgb", 1), ("right_camera_rgb", 2)):
        assert policy.observation[key].mean(axis=(0, 1)).argmax() == channel
    assert np.array_equal(policy.episode_cue, decoded["vision"]["cam_head"]["color"])


def test_instruction_list_and_missing_instruction(policy):
    obs = observation()
    obs["instructions"] = [obs.pop("instruction")]
    policy.update_obs(obs)
    assert policy.instruction == "Place the object in the box."
    obs.pop("instructions")
    with pytest.raises(ValueError, match="instruction"):
        policy.update_obs(obs)


def test_no_actions_before_first_observation(policy):
    with pytest.raises(RuntimeError, match="update_obs"):
        policy.get_action()


@pytest.mark.parametrize("kind", ["shape", "nan"])
def test_invalid_predictions_surface(policy, kind):
    policy.update_obs(observation())
    if kind == "shape":
        policy.policy.actions = np.zeros((31, 14), dtype=np.float32)
    else:
        policy.policy.actions[0, 0] = np.nan
    with pytest.raises(ValueError, match="Invalid predicted"):
        policy.get_action()


def test_unsupported_batch_is_explicit(policy):
    with pytest.raises(NotImplementedError):
        policy.update_obs_batch([observation()])
    with pytest.raises(NotImplementedError):
        policy.get_action_batch([0])


def test_loader_uses_public_namespace_and_asset_variables(monkeypatch, tmp_path):
    runtime, assets, backend = (tmp_path / name for name in ('runtime', 'assets', 'backend'))
    for path in (runtime, assets, backend):
        path.mkdir()
    constructor = Mock(return_value=object())
    serving = SimpleNamespace(__file__=str(runtime / 'wam/model/pixelwam/serving.py'), Policy=constructor)
    imported = Mock(return_value=serving)
    monkeypatch.setattr(adapter.importlib, 'import_module', imported)
    monkeypatch.setattr(adapter.sys, 'path', adapter.sys.path.copy())
    monkeypatch.setenv('PIXELWAM_MODEL_PATH', '')
    monkeypatch.setenv('PIXELWAM_SOURCE_PATH', '')
    config = dict(runtime_root=str(runtime), model_assets_path=str(assets),
                  backend_source_path=str(backend), weights_file='model.pt',
                  device='cpu', num_inference_steps=10, seed=0)
    adapter._load_policy(config, tmp_path)
    imported.assert_called_once_with('pixelwam.serving')
    assert adapter.os.environ['PIXELWAM_MODEL_PATH'] == str(assets)
    assert adapter.os.environ['PIXELWAM_SOURCE_PATH'] == str(backend)
    assert constructor.call_args.kwargs['checkpoint_path'] == str(tmp_path / 'model.pt')
    assert constructor.call_args.kwargs['config_path'] == str(tmp_path / 'config.yaml')


@pytest.mark.parametrize("encoded", [False, True])
def test_official_websocket_and_deploy_loop_with_mock_engine(policy, encoded):
    from client_server.ws.model_client import WsModelClient
    from client_server.ws.model_server import PolicyServer, PolicyServerConfig
    eval_one_episode = importlib.import_module("XPolicyLab.policy.Liber-0.deploy").eval_one_episode

    class Environment:
        step = 0

        def is_episode_end(self):
            return self.step >= 18

        def get_obs(self):
            obs = observation(10 + self.step)
            if encoded:
                for view in obs["vision"].values():
                    view["color"] = encode_image_bit(view["color"])
            return obs

        def take_action(self, action):
            assert len(action) == 4
            self.step += 1

    async def exercise():
        server = PolicyServer(policy, PolicyServerConfig(host="127.0.0.1", port=0))
        await server.start()
        port = server._server.sockets[0].getsockname()[1]

        def run_client():
            client = WsModelClient(url=f"ws://127.0.0.1:{port}", evaluation_id="mock-contract",
                                   trial_id="episode", max_connect_attempts=1)
            env = Environment()
            try:
                eval_one_episode(env, client)
            finally:
                client.close()
            assert env.step == 18

        try:
            await asyncio.to_thread(run_client)
        finally:
            await server.stop()

    asyncio.run(exercise())
    assert len(policy.policy.calls) == 2
    assert policy.replan_index == 2
    assert np.all(policy.episode_cue == 10)
